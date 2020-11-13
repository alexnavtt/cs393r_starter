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
#include <map>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "latency_compensator.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "vector_map/vector_map.h"
#include "global_planner.h"
#include "local_planner.h"
#include "nav_types.h"  // contains obstacle and path definitions

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {


class Navigation {
 public:

  /* -------- General Navigation Functions ---------- */
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
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
   // Main function called continously from main
  void Run();


  /* -------- Local Planner Functions ---------- */

  // Ackermann functions
  amrl_msgs::AckermannCurvatureDriveMsg AckermannFK(float x_dot, float y_dot, float omega);
  geometry_msgs::Twist AckermannIK(float curvature, float velocity);
  // Set and get the time before old obstacles are pruned off
  void setObstacleMemory(float delay);
  float getObstacleMemory();
  // Set and get the weights for the local planner cost function
  void setLocalPlannerWeights(float w_FPL, float w_C, float w_DTG);
  // Scale velocities to stay withing acceleration limits
  float limitVelocity(float vel);
  // Move along a given path
  void moveAlongPath(PathOption path);
  // Publish a drive command
  void driveCar(float curvature, float velocity);
  // Check to see if the nav goal has been reached
  void checkReached();

  /* -------- Helper Functions ---------- */
  Eigen::Vector2f BaseLink2Odom(Eigen::Vector2f p);
  Eigen::Vector2f Odom2BaseLink(Eigen::Vector2f p);
  void printVector(Eigen::Vector2f print_vector, std::string vector_name);


  /* -------- Visualization Functions -------- */
  void showObstacles();

 private:

  /* -------- Navigation Objects -------- */
 
  LatencyCompensator LC_;
  LocalPlanner local_planner_;
  GlobalPlanner global_planner_;

  /* ----------- Robot State ------------ */

  // Current robot location
  Eigen::Vector2f robot_loc_;
  // Current robot orientation
  float robot_angle_;
  // Current robot velocity
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed
  float robot_omega_;
  // Odometry-reported robot location
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle
  float odom_angle_;
  // Add a latency compensator
  

  /* ------ Frame Transformations ------ */
  Eigen::Matrix2f R_odom2base_;
  Eigen::Matrix2f R_map2odom_;

  /* --------- Navigation Planning --------- */
  // Whether navigation is complete
  bool nav_complete_;
  // Navigation goal location
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle
  float nav_goal_angle_;

  /* --------- Obstacles ---------- */

  // List of all obstacles in memory
  std::list<Obstacle> ObstacleList_;
  std::list<Obstacle> BaseLinkObstacleList_;
  float obstacle_memory_;  

  // Remove from memory any old or deprecated obstacles - called by ObservePointCloud
  void trimObstacles(double now);

};

}  // namespace navigation

#endif  // NAVIGATION_H
