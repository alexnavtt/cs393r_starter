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
#include <list>
#include <map>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "latency_compensator.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"

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
  float distance_to_goal;
  float cost;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  Eigen::Vector2f end_point; 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Obstacle{
  Eigen::Vector2f loc;
  double timestamp;
};

struct Neighbor{
  Eigen::Vector2i node_index;
  std::string key;
  float path_length;
  int neighbor_index;
};

struct Node{
  Eigen::Vector2f loc;                  // Location of node
  Eigen::Vector2i index;                // Index of node
  float cost;                           // Total path cost up to this node (NOTE: not edge cost)
  std::string parent;                   // Parent of the node on the optimal path                
  std::vector<Neighbor> neighbors;      // List of all valid adjacent nodes
  std::string key;                      // Unique identifier
};

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
  std::vector<float> getLocalPlannerWeights();
  // Visualize the current possible paths from the local planner
  void showLocalPaths();
  void showObstacles();
  // Scale velocities to stay withing acceleration limits
  float limitVelocity(float vel);
  // Move along a given path
  void moveAlongPath(PathOption path);
  // Publish a drive command
  void driveCar(float curvature, float velocity);
  // Get the best path towards the goal
  PathOption getGreedyPath(Eigen::Vector2f goal_loc);


  /* -------- Global Planner Functions ---------- */
  // Initialize the navigation mao at the start point and update the planner resolution
  void initializeMap(Eigen::Vector2f start_loc, float resolution);
  // Instantiate a new node as a child of another node
  Node newNode(const Node &old_node, int neighbor_index);
  // Check if travel from Node A to Node B is valid
  bool isValidNeighbor(const Node &node, const Neighbor &neighbor);
  // Find the travel cost bewteen two nodes
  float edgeCost(const Node &node_A,const Node &node_B);
  // Update valid neighbors and edge costs
  void visitNode(Node &node);
  // Get the best sequesnce of nodes to the nav_goal_ point
  std::vector<Node> getGlobalPath();


  /* -------- Helper Functions ---------- */
  Eigen::Vector2f BaseLink2Odom(Eigen::Vector2f p);
  Eigen::Vector2f Odom2BaseLink(Eigen::Vector2f p);
  void printPathDetails(PathOption path);
  void printVector(Eigen::Vector2f print_vector, std::string vector_name);
  std::string getNewID(int xi, int yi);
  std::vector<Neighbor> getNeighbors(const Node &node);



  /* -------- Visualization Functions -------- */
  void plotPathDetails(PathOption path);
  void plotNodeNeighbors(const Node &node);
  void visualizeMap();

 private:

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
  LatencyCompensator LC_;

  /* ------ Frame Transformations ------ */
  Eigen::Matrix2f R_odom2base_;
  Eigen::Matrix2f R_map2odom_;

  /* --------- Global Planning ---------- */

  // Whether navigation is complete
  bool nav_complete_;
  // Navigation goal location
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle
  float nav_goal_angle_;
  // Navigation map
  std::map<std::string, Node> nav_map_;
  float map_resolution_;
  // Priority Queue
  std::list<Node> frontier_;

  /* --------- Local Planning ---------- */

  // List of all obstacles in memory
  std::list<Obstacle> ObstacleList_;
  // List of all paths being considered
  std::vector<PathOption> PossiblePaths_;
  // Weight associated with how far the robot is able to travel on a path
  float free_path_length_weight_;
  // Wieght associated with how far the robot is from the nearest obstacle on a path
  float clearance_weight_;
  // Weight associated with how close the robot comes to the goal points on a path
  float distance_to_goal_weight_;
  // Time to keep old obstacles in seconds
  float obstacle_memory_;  
  // Obstacle Clearnace
  float clearance_;

  // Remove from memory any old or deprecated obstacles - called by ObservePointCloud
  void trimObstacles(double now);

  // Called by getGreedyPath
  void createPossiblePaths(float num);
  void predictCollisions(PathOption &path);
  void calculateClearance(PathOption &path);
  void trimPathLength(PathOption &path, Eigen::Vector2f goal);

  // LocalPlanner planner_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
