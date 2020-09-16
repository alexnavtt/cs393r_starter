//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "stdio.h" // Mark added

#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <string>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

// Note: namespace allows for "global" variables that exist only in this file   -Alex
namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// Delta t of the control loop
const float dt_ = 1/20.0;

// Robot Parameters
const float wheelbase_ = 0.324;
const float observation_delay_ = 0.0;
const float actuation_delay_ = 0.0;
const float vision_angle_ = 3*M_PI/2;  
const float vision_range_ = 10; // based on sim, grid squares are 2m
const float curvature_max_ = 1/1.0;

// Fake Robot Parameters
const float w = 0.25;	//width
const float l = 0.4;	//length
const float m = 0.0;	//padding
const float b = 0.3;	//wheelbase
const Vector2f pmin(0, w/2+m);
const Vector2f pdiff((b+l)/2+m, w/2+m);
const Vector2f pmax((b+l)/2+m, -w/2-m);

// Robot Limits
const float max_vel_   =  1.0;
const float min_vel_   = -1.0;
const float max_accel_ =  4.0;
const float min_accel_ = -4.0;

// Variables to deal with this dumb way of doing things
Eigen::Vector2f start_point_;

bool init_ = true;
} //namespace

namespace navigation {

bool isBetween(const Vector2f p0, const Vector2f p1, const Vector2f p2, const Vector2f p)
{
	/* Determine if p is inside the cone defined by p0, p1, and p2 
	    p2
	   /
	  /
	p0
	  \      p
	   \
	    p1
	*/

	float angle_p1 = atan2(p1[1] - p0[1], p1[0] - p0[0]);
	float angle_p2 = atan2(p2[1] - p0[1], p2[0] - p0[0]);
	float angle_p  = atan2( p[1] - p0[1],  p[0] - p0[0]);

	if (angle_p1 > 0 and angle_p2 < 0){
		return (angle_p > angle_p1 or angle_p1 < angle_p2);
	}
	else{
		return (angle_p > angle_p1 and angle_p < angle_p2);
	}
}

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
		robot_loc_(0, 0),
		robot_angle_(0),
		robot_vel_(0, 0),
		robot_omega_(0),
		LC_(actuation_delay_, observation_delay_, dt_),
		nav_complete_(true),
		nav_goal_loc_(0, 0),
		nav_goal_angle_(0),
		free_path_length_weight_(1),
		clearance_weight_(1),
		distance_to_goal_weight_(1),
		obstacle_memory_(10)
{
	drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
	viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
	local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
	global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
	InitRosHeader("base_link", &drive_msg_.header);
}

geometry_msgs::Pose2D Navigation::getOdomPose() const
{
	geometry_msgs::Pose2D pose;
	pose.x = odom_loc_[0];
	pose.y = odom_loc_[1];
	pose.theta = odom_angle_;
	return pose;
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
	nav_goal_loc_ = loc;
	nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
	robot_loc_ = loc;
	robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc, float angle,
								const Vector2f& vel, float ang_vel) {
	LC_.recordObservation(loc[0], loc[1], angle);
	state2D current_state = LC_.predictedState();

	odom_loc_ 	 = {current_state.x, current_state.y};
	odom_angle_  =  current_state.theta;
	robot_vel_ 	 = {current_state.vx, current_state.vy};
	robot_omega_ =  current_state.omega;

	R_odom2base_ << cos(odom_angle_), -sin(odom_angle_),
				    sin(odom_angle_),  cos(odom_angle_);

	init_ = false;
}

void Navigation::trimObstacles(double now)
{
	float upper_angle = odom_angle_ + 0.49*vision_angle_;
	float lower_angle = odom_angle_ - 0.49*vision_angle_;

	Vector2f lower_lim_point = BaseLink2Odom({cos(lower_angle), sin(lower_angle)});
	Vector2f upper_lim_point = BaseLink2Odom({cos(upper_angle), sin(upper_angle)});

	for (auto obs = ObstacleList_.begin(); obs != ObstacleList_.end();)	// Clarification: obs is of type std::list<Obstacle>::iterator
	{
		// If an obstacle is too old, get rid of it
		if (now - obs->timestamp > obstacle_memory_) {
			obs = ObstacleList_.erase(obs);
			continue;
		}
		
		// If it is within the field of view (i.e. we have new data) erase the old data
		if (isBetween(odom_loc_, lower_lim_point, upper_lim_point, obs->loc)){
			obs = ObstacleList_.erase(obs);
		}
		// Otherwise keep it in memory until it grows stale
		else{
			obs++;
		}
	}
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
	trimObstacles(time);

	for (auto &obs_loc : cloud)
	{
		ObstacleList_.push_back(Obstacle {BaseLink2Odom(obs_loc), time});
	}
}

void Navigation::showObstacles()
{
	int i = 0;
	int cutoff_count = ObstacleList_.size()/1000;
	for (const auto &obs : ObstacleList_)
	{
		if (i != cutoff_count) {i++; continue;} // ensure that no more than 1000 obstacles are displayed
		visualization::DrawCross(Odom2BaseLink(obs.loc), 0.1, 0x000000, local_viz_msg_);
		i = 0;
	}
}

void Navigation::createPossiblePaths(float num)
{
	PossiblePaths_.clear();

	float curve_increment = 2*curvature_max_/num;
	for (int i = 0; i < num; i++)
	{
		PossiblePaths_.push_back(PathOption {-curvature_max_ + i*curve_increment, 	// curvature
											0,										// clearance
											0,										// free path length
											{0,0},									// obstruction location
											{0,0},									// closest point location
											{0,0}});								// end point of the movement
	}
}

void Navigation::printVector(Vector2f print_vector, std::string vector_name)
{
	std::cout << vector_name << " x: " << print_vector.x() << ", y: " << print_vector.y() << std::endl;
}

// Calculate free path length for a given path
// Could expand this function to also return clearance, dist to goal, etc...
void Navigation::predictCollisions(PathOption& path){
	float r = 1/path.curvature; // turning radius
	Vector2f c(0,r); // point of rotation
	
	// Get radii to specific points based on curvature
	float rmin = (c-pmin).norm();
	float rdiff = (c-pdiff).norm();
	float rmax = (c-pmax).norm();

	// Minimum free path length so far as for loop executes
	float fpl_min = -1.0;	// If no obstacles, fpl will stay -1
	Vector2f p_closest(0,0);

	bool first_collision = true;
	// Iterate through points in point cloud
	for (const auto &obs : ObstacleList_)
	{
		// Grab point in the right frame
		Vector2f p_future = Odom2BaseLink(obs.loc);

		// Color points black as you go (removed for speed)
		// visualization::DrawCross(p_future, 0.1, 0x000000, local_viz_msg_);
		// viz_pub_.publish(local_viz_msg_);

		// Distance to point
		float rp = (c-p_future).norm();

		Vector2f p_current;
		// Check if point is hitting car on front or inner side
		if (rp > rmin && rp < rdiff) {
			// Inner side collision
			float phi = acos((r-w/2-m)/rp);
			float x = rp*sin(phi);
			p_current = {x, w/2+m};
		}
		else if (rp > rdiff && rp < rmax){
			// Front collision
			float phi = asin(((b+l)/2+m)/rp);
			float y = r-rp*cos(phi);
			p_current = {(b+l)/2+m, y};
		}
		else continue;
		
		// Use law of cosines to compute free path length
		float side_length = (p_future-p_current).norm();
		float theta = acos((side_length*side_length-2*rp*rp)/(-2*rp*rp));
		// Solve for free path length of the current point
		float fpl_current = theta*r;
		// If this is the first collision, record this as the smallest fpl so far
		if (first_collision){
			fpl_min = fpl_current;
			p_closest = p_future;
			first_collision = false;
		}
		// Only keep this value if it is the smallest fpl so far
		else if (fpl_current < fpl_min){
			fpl_min = fpl_current;
			p_closest = p_future;
		}
	}
	// Depict closest point with big green X
	visualization::DrawCross(p_closest, 0.5, 0x00ff00, local_viz_msg_);
	path.closest_point = p_closest;
	path.free_path_length = fpl_min;
}

void Navigation::calculateClearance(PathOption &path){
	// Ensure that a FPL has been calculated for the path already
	if (path.free_path_length == 0) {ROS_WARN("Calculating clearance before free path legnth. Returning 0!"); return;}

	// If radius is infinity (curvature == 0), make it some large number
	float turning_radius = (path.curvature == 0 ? 1e5 : 1/path.curvature);

	// Determine if the car is turning left or right (the math is slightly different for each)
	std::string turning_direction = (path.curvature <= 0 ? "right" : "left");

	// Find the geometry of the turning motion
	Vector2f turning_center = BaseLink2Odom({0, turning_radius});
	float turning_angle = path.free_path_length * path.curvature;

	Eigen::Matrix2f rotation;
	rotation << cos(turning_angle), -sin(turning_angle),
				sin(turning_angle),  cos(turning_angle);

	// Rotate odom_loc_ about turning_center for an angle of turning_angle to find the end_point
	Vector2f end_point = rotation * (odom_loc_ - turning_center) + turning_center;

	// // Define the cone the encompasses all the obstacles of interest
	Vector2f start_point = (turning_direction == "left" ? odom_loc_ : end_point);
	end_point 			 = (turning_direction == "left" ? end_point : odom_loc_);

	// Iterate through obstacles to find the one with the minimum clearance
	float min_clearance = vision_range_;
	Eigen::Vector2f closest_obs;
	for (const auto &obs : ObstacleList_)
	{
		if (isBetween(turning_center, start_point, end_point, obs.loc))
		{
			float clearance = abs( (obs.loc - turning_center).norm() - abs(turning_radius) );
			if (clearance < min_clearance) {min_clearance = clearance; closest_obs = obs.loc;}
		}
	}

	// Draw start and stop points relative to center
	visualization::DrawLine(Odom2BaseLink(turning_center), Odom2BaseLink(start_point), 0x000000, local_viz_msg_);
	visualization::DrawLine(Odom2BaseLink(turning_center), Odom2BaseLink(end_point), 0x000000, local_viz_msg_);
	visualization::DrawCross(Odom2BaseLink(turning_center), 0.15, 0xfc3003, local_viz_msg_);

	// Draw the closest obstacle which gives min clearance
	visualization::DrawCross(Odom2BaseLink(closest_obs), 0.15, 0x0bfc03, local_viz_msg_);

	path.clearance = min_clearance;
}


void Navigation::setLocalPlannerWeights(float w_FPL, float w_C, float w_DTG)
{
	free_path_length_weight_ = w_FPL;
	clearance_weight_ 		 = w_C;
	distance_to_goal_weight_ = w_DTG;
}

PathOption Navigation::getGreedyPath(Vector2f goal_loc)
{
	// Clear out possible paths and reinitialize
	createPossiblePaths(15);

	// Initialize output and cost
	PathOption BestPath;
	float min_cost = 1e10;

	// Iterate through paths to find the best one
	for (auto &path : PossiblePaths_)
	{
		// Update FLP, Clearance, Closest Point, Obstruction, End Point
		predictCollisions(path);

		float distance_to_goal = (path.end_point - goal_loc).norm();

		float cost = - path.free_path_length * free_path_length_weight_		// (-) decrease cost with large FPL
					 - path.clearance 		 * clearance_weight_			// (-) decrease cost with large clearance
					 + distance_to_goal 	 * distance_to_goal_weight_; 	// (+) increase cost with large distance to goal

		if (cost < min_cost) {min_cost = cost; BestPath = path;}
	}

	return BestPath;
}

// Limit Velocity to follow both acceleration and velocity limits
float Navigation::limitVelocity(float vel) {
	float new_vel = std::min({vel,     robot_vel_[0] + max_accel_ * dt_, max_vel_});
	return          std::max({new_vel, robot_vel_[0] + min_accel_ * dt_, min_vel_});
}

void Navigation::moveForwards(Vector2f& start, float dist){
	// Update how far you've come and how far to go
	float dist_traveled = (odom_loc_ - start).norm();
	float dist_to_go = dist - dist_traveled;

	// Update current velocity and solve for necessary stopping distance
	float current_speed = robot_vel_.norm();
	float decel_dist = -0.5*current_speed*current_speed/min_accel_;

	// Determine whether to accel or decel, then publish command
	float cmd_vel = (dist_to_go > decel_dist) ? max_vel_ : 0.0;
	driveCar(0.0, limitVelocity(cmd_vel));
}

void Navigation::moveCurvy(PathOption best_path){
	// Connor works here
	float current_speed = robot_vel_.norm();
	float decel_dist = -0.5*current_speed*current_speed/min_accel_;
	
	float cmd_vel = (best_path.free_path_length > decel_dist) ? max_vel_ : 0.0;
	driveCar(best_path.curvature, limitVelocity(cmd_vel));
}

void Navigation::driveCar(float curvature, float velocity){
	drive_msg_.header.seq++;
	drive_msg_.header.stamp = ros::Time::now();
	drive_msg_.curvature = curvature;
	drive_msg_.velocity = velocity;
	drive_pub_.publish(drive_msg_);

	// Record input in the latency compensator
	geometry_msgs::Twist cartesian_velocity = AckermannIK(curvature, velocity);
	LC_.recordNewInput(cartesian_velocity.linear.x, 
					   cartesian_velocity.linear.y, 
					   cartesian_velocity.angular.z);
}

// Ackerman Forward/Inverse Kinematics
AckermannCurvatureDriveMsg Navigation::AckermannFK(float x_dot, float y_dot, float omega){
	float theta = atan2(y_dot, x_dot);
	float velocity = x_dot*cos(theta);
	float curvature = omega/velocity;

	AckermannCurvatureDriveMsg ackermann_msg;
	ackermann_msg.velocity = velocity;
	ackermann_msg.curvature = curvature;
	return ackermann_msg;
}
geometry_msgs::Twist Navigation::AckermannIK(float curvature, float velocity){
	float theta = odom_angle_;
	float x_dot = velocity*cos(theta);
	float y_dot = velocity*sin(theta);
	float omega = velocity*curvature;

	geometry_msgs::Twist vel;
	vel.linear.x = x_dot;
	vel.linear.y = y_dot;
	vel.angular.z = omega;
	return vel;
}

// Frame Transformations
Eigen::Vector2f Navigation::BaseLink2Odom(Eigen::Vector2f p) {return odom_loc_ + R_odom2base_*p;}
Eigen::Vector2f Navigation::Odom2BaseLink(Eigen::Vector2f p) {return R_odom2base_.transpose()*(p - odom_loc_);}

// Main Loop
void Navigation::Run() {
	visualization::ClearVisualizationMsg(local_viz_msg_);
	visualization::ClearVisualizationMsg(global_viz_msg_);

	// Added this section for anything that we only want to happen once (like Arduino Setup function)
	if  (init_){
		while (init_ and ros::ok()){
			ros::spinOnce();
			ros::Rate(10).sleep();
		}

		start_point_ = odom_loc_;
	}

	// Pseudocode:
	// createPossiblePaths(5);
	// for each path:
	//   predictCollisions   -  gets free path length
	//   calculateClearance  -  calculates clearance, who woulda thunk
	//   assignCost
	// executeBestPath(path_with_lowest_cost)

	PathOption test_path{1e-5, 0, 0, {0,0}, {0,0}, {0,0}};	//10m radius
	predictCollisions(test_path);
	calculateClearance(test_path);
	std::cout << "free path length: " << test_path.free_path_length << std::endl;

	driveCar(test_path.curvature, 0.0);

	showObstacles();

	viz_pub_.publish(local_viz_msg_);
	viz_pub_.publish(global_viz_msg_);
}

}  // namespace navigation
