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

// Note: namespace allows for "global" variables that exist only in this file
namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// Delta t of the control loop
float dt_ = 1/20.0;	// made not constant to adapt for variance/lag
// NOTE: This may cause issues with the latency compensator

// Robot Parameters
const float wheelbase_ = 0.324;
const float observation_delay_ = 0.0;
const float actuation_delay_ = 0.0;
const float vision_angle_ = 3*M_PI/2;  
const float vision_range_ = 10; // based on sim, grid squares are 2m
const float curvature_max_ = 1/1.0;
const float max_clearance_ = 2.0;

// Fake Robot Parameters
const float w = 0.27;	//width
const float l = 0.5;	//length
const float m = 0.1;	//padding
const float b = 0.324;	//wheelbase
const Vector2f pmin(0, w/2+m);
const Vector2f pdif((b+l)/2+m, w/2+m);
const Vector2f pmax((b+l)/2+m, -w/2-m);

// Robot Limits
const float max_vel_   =  1.0;
const float min_vel_   = -1.0;
const float max_accel_ =  4.0;
const float min_accel_ = -4.0;

// Global one-time variables
Eigen::Vector2f goal_vector_;
bool navigation_success_ = false;
bool init_ = true;
bool init_vel_ = true;
ros::Time time_prev_;
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
		obstacle_memory_(0)
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
		float curvature = -curvature_max_ + i*curve_increment;
		// Enforce max radius of 1km (any bigger and the angles get so small the math is bad)
		if (std::abs(curvature) < 0.001) curvature = 0.001;
		PossiblePaths_.push_back(PathOption {curvature, // curvature
											 0,			// clearance
											 0,			// free path length
											 0,			// distance to goal
											 {0,0},		// obstruction location
											 {0,0},		// closest point location
											 {0,0}});	// end point of the movement
	}
}

void Navigation::printVector(Vector2f print_vector, std::string vector_name){
	std::cout << vector_name << "\t x= " << print_vector.x() << ", y= " << print_vector.y() << std::endl;
}

void Navigation::trimPathLength(PathOption &path, Vector2f goal)
{
	// NOTE: Defined in the Base Link frame
	Vector2f P_center = {0, 1/path.curvature};
	Vector2f V_center2goal = (goal - P_center) / (goal - P_center).norm();
	Vector2f V_center2home = -P_center/P_center.norm();

	float angle = acos(V_center2home.dot(V_center2goal));
	if (angle < 0) angle = M_PI + angle; 		// negative angle means it is obtuse
	if (goal[0] < 0) angle = 2*M_PI - angle; 	// negative x means angle > 180°

	path.free_path_length = std::min(path.free_path_length, abs(angle/path.curvature));
}

// Calculate free path length for a given path
void Navigation::predictCollisions(PathOption& path){
	float r = 1/path.curvature; // radius of curvature

	// Need to flip certain variables if turning CW
	int sign = ( (r > 0) ? 1 : -1);

	Vector2f c(0,r); // point of rotation
	
	// Get radii to specific points based on curvature
	float rmin = (sign*c-pmin).norm();
	float rdif = (sign*c-pdif).norm();
	float rmax = (sign*c-pmax).norm();

	// Initialize loop variables
	float fpl_min = 0.0;
	Vector2f p_obstruction(0.0, 0.0);

	bool first_loop = true;
	// Iterate through points in point cloud
	for (const auto &obs : ObstacleList_)
	{
		// Get point in the right frame
		Vector2f p_future = Odom2BaseLink(obs.loc);
		// Distance to point
		float rp = (c-p_future).norm();
		
		// Initialize loop variables
		float fpl_current;
		bool obstruction = false;

		// Check if point will obstruct car
		if (rp > rmin && rp < rmax) {
			obstruction = true;
			Vector2f p_current;
			if (rp < rdif) {
				// Inner side collision
				float phi = acos((sign*r-w/2-m)/rp);
				float x = rp*sin(phi);
				p_current = {x, sign*(w/2+m)};
			}
			else if (rp > rdif) {
				// Front collision
				float phi = asin(((b+l)/2+m)/rp);
				float y = r-sign*rp*cos(phi);
				p_current = {(b+l)/2+m, y};
			}
			// Use law of cosines to compute free path length
			float side_length = (p_future-p_current).norm();
			float theta = acos((side_length*side_length-2*rp*rp)/(-2*rp*rp));
			// Solve for free path length of the current point
			fpl_current = sign*theta*r;
		}
		else {
			// No collision
			// NOTE: This limiting method favors straight paths due to how the GLP is weighted
			fpl_current = sign*r*M_PI; // 180deg U-turn
			if (fpl_current > vision_range_) fpl_current = vision_range_; // limit to range of Lidar
		}
		
		// If this is the first loop or this is the smallest fpl so far, record this value
		if (first_loop || fpl_current < fpl_min){
			fpl_min = fpl_current;
			if (obstruction) p_obstruction = p_future;
			first_loop = false;
		}
	}
	// Save results to path struct
	path.obstruction = p_obstruction;
	path.free_path_length = fpl_min;
}

void Navigation::calculateClearance(PathOption &path){
	// Ensure that a FPL has been calculated for the path already
	// if (path.free_path_length == 0) {ROS_WARN("Calculating clearance before free path length. Returning 0!"); return;}

	// If radius is infinity (curvature == 0), make it some large number
	// float turning_radius = (path.curvature == 0 ? 1e5 : 1/path.curvature);
	float turning_radius = 1/path.curvature;
	float abs_radius = abs(turning_radius);

	// Determine if the car is turning left or right (the math is slightly different for each)
	std::string turning_direction = (path.curvature < 0 ? "right" : "left");

	// Find the geometry of the turning motion
	Vector2f turning_center = BaseLink2Odom({0, turning_radius});
	float turning_angle = path.free_path_length * path.curvature;

	Eigen::Matrix2f rotation;
	rotation << cos(turning_angle), -sin(turning_angle),
				sin(turning_angle),  cos(turning_angle);

	// Rotate odom_loc_ about turning_center for an angle of turning_angle to find the end_point
	// Vector2f end_point = rotation * (odom_loc_ - turning_center) + turning_center;
	// end_point = Odom2BaseLink(end_point);
	// end_point.x() += ((b+l)/2+m) * cos(turning_angle);
	// end_point.y() += ((b+l)/2+m) * sin(turning_angle);
	// path.end_point = end_point;
	// end_point = BaseLink2Odom(end_point);

	// NOTE: While that was fun, it is probably better to use the obstruction point
	//       which will also account for the part of the car ahead of the base_link
	Vector2f end_point = path.obstruction;

	// Define the cone that encompasses all the obstacles of interest
	Vector2f start_point = (turning_direction == "left" ? odom_loc_ : end_point);
	end_point 			 = (turning_direction == "left" ? end_point : odom_loc_);

	// Iterate through obstacles to find the one with the minimum clearance
	float min_clearance = vision_range_;
	Eigen::Vector2f closest_obs;
	for (const auto &obs : ObstacleList_)
	{
		if (isBetween(turning_center, start_point, end_point, obs.loc))
		{
			float offset = (obs.loc - turning_center).norm();
			float clearance = (offset < abs_radius) ? abs_radius - w/2.0 - m - offset : offset - (abs_radius + w/2.0 + m);
			clearance = std::max(0.0f, clearance);

			if (clearance < min_clearance) {
				min_clearance = clearance;
			}
		}
	}

	// Draw start and stop points relative to center
	// visualization::DrawLine(Odom2BaseLink(turning_center), Odom2BaseLink(start_point), 0x000000, local_viz_msg_);
	// visualization::DrawLine(Odom2BaseLink(turning_center), Odom2BaseLink(end_point), 0x000000, local_viz_msg_);
	// visualization::DrawCross(Odom2BaseLink(turning_center), 0.15, 0xfc3003, local_viz_msg_);

	// Draw the closest obstacle which gives min clearance
	// visualization::DrawCross(Odom2BaseLink(closest_obs), 0.15, 0x0bfc03, local_viz_msg_);
	path.closest_point = Odom2BaseLink(closest_obs);
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
	createPossiblePaths(20);

	// Initialize output and cost
	PathOption BestPath;
	float min_cost = 1e10;

	goal_loc = Odom2BaseLink(goal_loc);

	// Get best parameter from all possible paths for normalization
	float max_free_path_length = 1e-5;
	float max_clearance = 1e-5;
	float min_distance_to_goal = 1e5;
	for (auto &path : PossiblePaths_)
	{
		
		// Update FLP, Clearance, Closest Point, Obstruction, End Point
		// NOTE: Dont do this twice, horrible practice
		predictCollisions(path);
		trimPathLength(path, goal_loc);
		calculateClearance(path);

		if (path.free_path_length > max_free_path_length) max_free_path_length = path.free_path_length;
		if (path.clearance > max_clearance) max_clearance = path.clearance;
		float distance_to_goal = (path.end_point - goal_loc).norm();
		path.distance_to_goal = distance_to_goal;
		if (distance_to_goal < min_distance_to_goal) min_distance_to_goal = distance_to_goal;
	}
	// std::cout << "fpl_b: " << max_free_path_length << ", c_b: " << max_clearance << ", dtg_b: " << min_distance_to_goal << std::endl;

	// Iterate through paths to find the best one
	for (auto &path : PossiblePaths_)
	{
		// Update FLP, Clearance, Closest Point, Obstruction, End Point
		predictCollisions(path);
		trimPathLength(path, goal_loc);
		calculateClearance(path);
		
		// consider changing clearance to:
		// + ( max_clearance / path.clearance )
		float cost = - ( path.free_path_length / max_free_path_length ) * free_path_length_weight_		// (-) decrease cost with large FPL
					 - ( path.clearance / max_clearance )		  		* clearance_weight_				// (+) increase cost with small clearance
					 + ( path.distance_to_goal / min_distance_to_goal ) * distance_to_goal_weight_; 	// (+) increase cost with large distance to goal
		if ( path.clearance < m ) cost = cost*5; // significantly increase cost if not enough clearance
		// std::cout << "curvature: " << path.curvature << ", cost: " << cost << std::endl;

		if (cost < min_cost) {min_cost = cost; BestPath = path;}
	}

	return BestPath;
}

// Limit Velocity to follow both acceleration and velocity limits
float Navigation::limitVelocity(float vel) {
	// For some very strange reason, the y-term of velocity is initialized at infinity...
	float current_speed = robot_vel_.norm();
	if (init_vel_) { current_speed = 0; init_vel_ = false; }
	ros::Time time_now = ros::Time::now();
	ros::Duration delta_time = time_now - time_prev_;
	dt_ = delta_time.toSec();
	// Changed to account for 2d speed
	float new_vel = std::min({vel, 		current_speed + max_accel_ * dt_, max_vel_});
	float cmd_vel = std::max({new_vel, 	current_speed + min_accel_ * dt_, min_vel_});
	time_prev_ = time_now;
	return cmd_vel;
}

void Navigation::moveAlongPath(PathOption path){
	float current_speed = robot_vel_.norm();
	float decel_dist = 0.1+-0.5*current_speed*current_speed/min_accel_;
	float cmd_vel = (path.free_path_length > decel_dist) ? max_vel_ : 0.0;
	// if (navigation_success_) cmd_vel = 0;
	driveCar(path.curvature, limitVelocity(cmd_vel));
}

void Navigation::plotPathDetails(PathOption path){
	// Outline car
	visualization::DrawLine({-(l-b)/2, w/2},  {(b+l)/2, w/2},  0x000000, local_viz_msg_);	//top
	visualization::DrawLine({-(l-b)/2, -w/2}, {(b+l)/2, -w/2}, 0x000000, local_viz_msg_);	//bottom
	visualization::DrawLine({(b+l)/2, w/2},   {(b+l)/2, -w/2}, 0x000000, local_viz_msg_);	//front
	visualization::DrawLine({-(l-b)/2, -w/2}, {-(l-b)/2, w/2}, 0x000000, local_viz_msg_);	//back

	// Place green cross at obstruction point if it exists
	if (path.obstruction.norm() > 0)
		visualization::DrawCross(path.obstruction, 0.5, 0x00ff00, local_viz_msg_);

	// Draw all the possible path options
	for (auto other_path : PossiblePaths_)
	{
		other_path.clearance = 0;
		visualization::DrawPathOption(other_path.curvature, other_path.free_path_length, other_path.clearance, local_viz_msg_);
	}

	// Draw path option
	visualization::DrawPathOption(path.curvature, path.free_path_length, path.clearance, local_viz_msg_);
	
	// Place red cross at goal position
	visualization::DrawCross(Odom2BaseLink(goal_vector_), 0.5, 0xff0000, local_viz_msg_);
}

void Navigation::printPathDetails(PathOption path){
	std::cout << "loop time:        " << dt_ << std::endl
			  << "current velocity: " << robot_vel_.norm() << std::endl
			  << "curvature:        " << path.curvature << std::endl
			  << "clearance:        " << path.clearance << std::endl
			  << "free path length: " << path.free_path_length << std::endl
			  << "distance_to_goal: " << path.distance_to_goal << std::endl;
	printVector(path.obstruction, "obstruction: ");
	printVector(path.closest_point, "closest point: ");
	printVector(path.end_point, "end point: ");
	printVector(goal_vector_, "goal position");
	std::cout << " - - - - - - - - - - - - - - - " << std::endl;
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
		Vector2f goal(10,0);
		goal_vector_ = BaseLink2Odom(goal);
		// Set cost function weights (FPL, clearance, distance to goal)
		setLocalPlannerWeights(4,1,0);
		time_prev_ = ros::Time::now();
	}

	showObstacles();

	PathOption BestPath = getGreedyPath(goal_vector_);
	moveAlongPath(BestPath);
	printPathDetails(BestPath);
	plotPathDetails(BestPath);

	// If we have reached our goal we can stop
	float dist_to_goal = (odom_loc_-goal_vector_).norm();
	float current_speed = robot_vel_.norm();
	if (current_speed > 2.0) current_speed = 0; // disregards initial infinite velocity
	float stopping_dist = 0.1+-0.5*current_speed*current_speed/min_accel_;
	if (dist_to_goal <= stopping_dist){
		navigation_success_ = true;
		ROS_WARN("Navigation success!");
	}

	// PathOption test_path{0.001, 0, 0, 0, {0,0}, {0,0}, {0,0}};
	// predictCollisions(test_path);
	// std::cout << "free path length: " << test_path.free_path_length << std::endl;
	// plotPathDetails(test_path);

	viz_pub_.publish(local_viz_msg_);
	viz_pub_.publish(global_viz_msg_);
}

}  // namespace navigation
