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
#include "stdio.h"

#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using visualization::DrawPoint;
using visualization::DrawCross;
using visualization::DrawLine;

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

// // Robot Parameters
const float observation_delay_ 	= 0.0;
const float actuation_delay_ 	= 0.0;
const float vision_angle_ 		= 3*M_PI/2;  
const float vision_range_ 		= 10; 		// based on sim, grid squares are 2m

// Robot Limits
const float max_vel_   =  1.0;
const float min_vel_   = -1.0;
const float max_accel_ =  4.0;
const float min_accel_ = -4.0;

// Global one-time variables
Eigen::Vector2f local_goal_vector_;
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
		return (angle_p > angle_p1 or angle_p < angle_p2);
	}
	else{
		return (angle_p > angle_p1 and angle_p < angle_p2);
	}
}

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
		LC_(actuation_delay_, observation_delay_, dt_),
		robot_loc_(0, 0),
		robot_angle_(0),
		robot_vel_(0, 0),
		robot_omega_(0),
		nav_complete_(true),
		nav_goal_loc_(0, 0),
		// nav_goal_angle_(0),
		obstacle_memory_(1)
{
	global_planner_.setResolution(0.25);
	setLocalPlannerWeights(2,1,2); //fpl, clearance, dtg

	drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
	viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
	local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
	global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
	InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
	nav_goal_loc_ = loc;
	nav_goal_angle_ = angle;
	// std::cout << "Test" << std::endl;

	global_planner_.initializeMap(robot_loc_); 
	global_planner_.getGlobalPath(nav_goal_loc_);

	nav_complete_ = false;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
	robot_loc_ = loc;
	robot_angle_ = angle;

	R_map2base_ << cos(angle), -sin(angle),
			       sin(angle),  cos(angle);
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

	auto baseLinkObsPtr = BaseLinkObstacleList_.begin();
	for (auto obs = ObstacleList_.begin(); obs != ObstacleList_.end();)	// Clarification: obs is of type std::list<Obstacle>::iterator
	{
		// If an obstacle is too old, get rid of it
		if (now - obs->timestamp > obstacle_memory_) {
			obs = ObstacleList_.erase(obs);
			baseLinkObsPtr = BaseLinkObstacleList_.erase(baseLinkObsPtr);
			continue;
		}
		
		// If it is within the field of view (i.e. we have new data) erase the old data
		if (isBetween(odom_loc_, lower_lim_point, upper_lim_point, obs->loc)){
			obs = ObstacleList_.erase(obs);
			baseLinkObsPtr = BaseLinkObstacleList_.erase(baseLinkObsPtr);
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
		BaseLinkObstacleList_.push_back(Obstacle {obs_loc, time});
	}
}

void Navigation::showObstacles()
{
	int i = 0;
	int cutoff_count = ObstacleList_.size()/1000;
	for (const auto &obs : ObstacleList_)
	{
		if (i != cutoff_count) {i++; continue;} // ensure that no more than 1000 obstacles are displayed
		visualization::DrawCross(Odom2BaseLink(obs.loc), 0.05, 0x000000, local_viz_msg_);
		i = 0;
	}
}

void Navigation::setLocalPlannerWeights(float w_FPL, float w_C, float w_DTG)
{
	local_planner_.setWeights(w_FPL, w_C, w_DTG);
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

void Navigation::printVector(Vector2f print_vector, string vector_name){
	std::cout << vector_name << "\t x= " << print_vector.x() << ", y= " << print_vector.y() << std::endl;
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
// FK never gets used
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

void Navigation::checkReached(){
	// If we have reached our goal we can stop (not relevant for dynamic goal)
	float dist_to_goal = local_goal_vector_.norm();
	float current_speed = robot_vel_.norm();
	if (current_speed > 2.0) return; // disregards initial infinite velocity
	float stopping_dist = 0.1 - 0.5*current_speed*current_speed/min_accel_;

	if (dist_to_goal <= stopping_dist){
		nav_complete_ = true;
		ROS_WARN("Navigation success!");
	}
}

void Navigation::checkStalled(){
	if (not stalled_ and drive_msg_.velocity < 0.01){
		stalled_ = true;
		stall_time_ = ros::Time::now();
	}else if(drive_msg_.velocity > 0.01){
		stalled_ = false;
	}
}

bool Navigation::isRobotStuck(){
	ros::Time now = ros::Time::now();
	if (stalled_ and (now - stall_time_).toSec() > 5){
		return true;
	}
	return false;
}

// Frame Transformations
Eigen::Vector2f Navigation::BaseLink2Odom(Eigen::Vector2f p) {return odom_loc_ + R_odom2base_*p;}
Eigen::Vector2f Navigation::Odom2BaseLink(Eigen::Vector2f p) {return R_odom2base_.transpose()*(p - odom_loc_);}
Eigen::Vector2f Navigation::Map2BaseLink(Eigen::Vector2f p) {return R_map2base_.transpose()*(p - robot_loc_);}


// Main Loop
void Navigation::Run() {
	visualization::ClearVisualizationMsg(local_viz_msg_);
	visualization::ClearVisualizationMsg(global_viz_msg_);

	if (nav_complete_){
		// Do nothing is navigation is not active
		ros::Duration(0.5).sleep();

	}else{
		// Extract the next node to aim for by the local planner
		Node target_node = global_planner_.getClosestPathNode(robot_loc_, global_viz_msg_);
		local_goal_vector_ = Map2BaseLink(target_node.loc);
		

		// Find the greedy local path to this point
		PathOption BestPath = local_planner_.getGreedyPath(local_goal_vector_, BaseLinkObstacleList_);
		moveAlongPath(BestPath);
		checkReached();

		checkStalled();
		if (global_planner_.needsReplan() or isRobotStuck()){
			global_planner_.replan(robot_loc_, target_node.loc);
			cout << "Replan!" << endl;
		}

		// Visualization/Diagnostics
		// local_planner_.printPathDetails(BestPath, local_goal_vector_);
		local_planner_.plotPathDetails(BestPath, local_goal_vector_, local_viz_msg_);
		// cout << nav_complete_ << endl;
	}

	// Visualization
	global_planner_.plotGlobalPath(global_viz_msg_);
	global_planner_.plotFrontier(global_viz_msg_);

	viz_pub_.publish(local_viz_msg_);
	viz_pub_.publish(global_viz_msg_);
}

}  // namespace navigation
