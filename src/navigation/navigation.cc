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
#include "ros/console.h" // Mark added
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

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

// Robot Limits
const float max_vel_   =  1.0;
const float min_vel_   = -1.0;
const float max_accel_ =  4.0;
const float min_accel_ = -4.0;

// Minimum distance needed for the car to accelerate/decelerate fully
const float accel_dist_ =  0.5*max_vel_*max_vel_/max_accel_;
const float decel_dist_ = -0.5*max_vel_*max_vel_/min_accel_;

// Variables to deal with this dumb way of doing things
Eigen::Vector2f start_point_;

bool init_ = true;
} //namespace

namespace navigation {

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
		obstacle_memory_(5)
{
	drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
	viz_pub_   = n->advertise<VisualizationMsg>("visualization", 1);
	local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
	global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
	InitRosHeader("base_link", &drive_msg_.header);
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
	// odom_loc_ = loc;
	// odom_angle_ = angle;
	// robot_vel_ = vel;
	// robot_omega_ = ang_vel;

	LC_.recordObservation(loc[0], loc[1], angle);
	state2D current_state = LC_.predictedState();

	odom_loc_ 	 = {current_state.x, current_state.y};
	odom_angle_  =  current_state.theta;
	robot_vel_ 	 = {current_state.vx, current_state.vy};
	robot_omega_ =  current_state.omega;

	init_ = false;
}

void Navigation::trimObstacles(double now)
{
	float upper_angle = odom_angle_ + 0.5*vision_angle_;
	float lower_angle = odom_angle_ - 0.5*vision_angle_;

	for (auto obs = ObstacleList_.begin(); obs != ObstacleList_.end();)	// Clarification: obs is of type std::list<Obstacle>::iterator
	{
		// If an obstacle is too old, get rid of it
		if (now - obs->timestamp > obstacle_memory_) {
			ObstacleList_.erase(obs);
			continue;
		}

		// Check to see where in the reference frame obs lies relative to the robot
		float obs_angle = atan2(obs->loc[1] - odom_loc_[1], obs->loc[0] - odom_loc_[0]);  				// in the range [-pi, pi]

		// If it is within the field of view (i.e. we have new data) erase the old data
		if (obs_angle < upper_angle and obs_angle > lower_angle){
			ObstacleList_.erase(obs);
		}
		// Otherwise keep it in memory until it grows stale
		else{
			obs++;
		}
	}
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
	trimObstacles(time);

	for (const auto &obs : cloud)
	{
		ObstacleList_.push_back(Obstacle {obs, time});
	}
}

// Return free path length of a given path (WIP)
float Navigation::predictCollisions(PathOption path){
	free_path_length = vision_range_;

	return free_path_length;
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

void Navigation::Run() {
	// Added this section for anything that we only want to happen once (like Arduino Setup function)
	if  (init_){
		while (init_ and ros::ok()){
			ros::spinOnce();
			ros::Rate(10).sleep();
		}

		start_point_ = odom_loc_;
	}

	// Drive forwards 1 meter from start point
	moveForwards(start_point_, 10.0);
	// driveCar(0.0, 1.0);
}

}  // namespace navigation
