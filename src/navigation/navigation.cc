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
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
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

// Robot Limits
const float max_vel_   =  1.0;
const float min_vel_   = -1.0;
const float max_accel_ =  4.0;
const float min_accel_ = -4.0;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
        robot_loc_(0, 0),
        robot_angle_(0),
        robot_vel_(0, 0),
        robot_omega_(0),
        nav_complete_(true),
        nav_goal_loc_(0, 0),
        nav_goal_angle_(0) {
    drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
            "ackermann_curvature_drive", 1);
    viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
    local_viz_msg_ = visualization::NewVisualizationMessage(
            "base_link", "navigation_local");
    global_viz_msg_ = visualization::NewVisualizationMessage(
            "map", "navigation_global");
    InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
}

void Navigation::UpdateOdometry(const Vector2f& loc, float angle,
                                const Vector2f& vel, float ang_vel) {
    robot_loc_ = loc;
    robot_angle_ = angle;
    robot_vel_ = vel;
    robot_omega_ = ang_vel;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                                                     double time) {
}

// New function I added     -Alex
float Navigation::limitVelocity(float vel){
		float new_vel = std::min({vel,     robot_vel_[0] + max_accel_*dt_, max_vel_});
    return          std::max({new_vel, robot_vel_[0] + min_accel_*dt_, min_vel_});
}

void Navigation::Run() {
    drive_msg_.header.seq++;
    drive_msg_.header.stamp = ros::Time::now();
		if (robot_loc_[0] < 1.0)
		{
			drive_msg_.velocity = limitVelocity(1.0);
		}
		else
		{
			drive_msg_.velocity = limitVelocity(0.0);
		}
    drive_msg_.curvature = 0.0;

    drive_pub_.publish(drive_msg_);
    // Create Helper functions here
    // Milestone 1 will fill out part of this class.
    // Milestone 3 will complete the rest of navigation.
}

}  // namespace navigation
