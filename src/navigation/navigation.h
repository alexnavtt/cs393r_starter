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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>
#include <algorithm>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose2D.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  // Scale velocities to stay withing acceleration limits
  float limitVelocity(float vel);

  // Jennings: Added this to move forwards a set distance in x direction
  void moveForwards(float start, float dist);

  // Publish a drive command
  void driveCar(float curvature, float velocity);

 private:

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
};


class LatencyCompensator {
public:
  LatencyCompensator(float actuation_delay, float observation_delay);
  void recordNewInput(float curvature, float velocity);
  geometry_msgs::Pose2D predictedState();

private:
  std::vector< std::array<float,3> > recordedInputs_; // Record of past inputs in the form (curvature, speed, timestamp)
  float actuation_delay_;                             // Robot actuation delay (seconds)
  float observation_delay_;                           // Sensor observation delay (seconds)
  float system_delay_;                                // Total system delay (ignoring contoller delay) (seconds)
  float last_observation_time_;                       // Timestamp for when the last sensor state came in 
  float delta_t_;                                      // Duration of a control loop for the system being compensated

  void trimInputs();
};

}  // namespace navigation

#endif  // NAVIGATION_H
