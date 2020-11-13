#ifndef LOCAL_PLANNER_HH
#define LOCAL_PLANNER_HH

#include <vector>
#include <list>
#include "geometry_msgs/Pose2D.h"
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "visualization/visualization.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "geometry_msgs/Twist.h"
#include "nav_types.h"

namespace navigation{

class LocalPlanner{
public:
	// Constructor
	LocalPlanner();

	// Set the weights for the local planner cost function
	void setWeights(float w_FPL, float w_C, float w_DTG);
	// Get the best path towards the goal
	PathOption getGreedyPath(Eigen::Vector2f goal_loc, const std::list<Obstacle> &obstacle_list);

	/* -------- Helper Functions ---------- */
	void printPathDetails(PathOption path, Eigen::Vector2f goal_loc);

	/* -------- Visualization Functions -------- */
	void plotPathDetails(PathOption path, Eigen::Vector2f goal_loc, amrl_msgs::VisualizationMsg &msg);

private:

	/* ----- Helper Functions ----- */

	// Called by getGreedyPath
	void createPossiblePaths(int num);
	void predictCollisions(PathOption& path, const Eigen::Vector2f goal_loc, const std::list<Obstacle> &obstacle_list);
	void calculateClearance(PathOption& path, const std::list<Obstacle> &obstacle_list);
	void trimPathLength(PathOption &path, Eigen::Vector2f goal);


	/* --- Private Members --- */

	// Car geometry
	float car_width_;	
	float car_length_;
	float padding_;	
	float wheelbase_;	
	Eigen::Vector2f pmin_;
	Eigen::Vector2f pdif_;
	Eigen::Vector2f pmax_;

	// Planner Parameters
	float observation_delay_;
	float actuation_delay_;
	float vision_angle_;  
	float vision_range_; 		// based on sim, grid squares are 2m
	float curvature_max_;		// can take turns as tight as 1m
	float clearance_limit_;
	
	// Paths
	std::vector<PathOption> PossiblePaths_;
	float free_path_length_weight_;
	float clearance_weight_;
	float distance_to_goal_weight_;
};

} // namespace navigation

#endif
