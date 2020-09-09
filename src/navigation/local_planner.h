#ifndef LOCAL_PLANNER_HH
#define LOCAL_PLANNER_HH

#include <vector>
#include "geometry_msgs/Pose2D.h"
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"

struct PathOption {
	float curvature;
	float clearance;
	float free_path_length;
	Eigen::Vector2f obstruction;
	Eigen::Vector2f closest_point;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Obstacle{
	Eigen::Vector2f loc;
	ros::Time stamp;
};

class localPlanner{
public:
	// Constructor
	localPlanner();

	// Public API
	void updateState(float x, float y, float theta);
	void setGreedyPlannerWeights(float w_FPL, float w_C, float w_DTG);
	void importObstacles(std::vector<Eigen::Vector2f> &obstacles);
	PathOption getGreedyPlannerPath();

private:
	// Called by importOstacles
	void trimObstacles();

	// Called by getGreedyPlannerPath
	void createPossiblePaths();
	void predictCollisions();
	void showPaths() const;

	// Private Members
	std::vector<Obstacle> ObstacleList;
	std::vector<PathOption> PossiblePaths;

	float free_path_length_weight_;
	float clearance_weight_;
	float distance_to_goal_weight_;

	geometry_msgs::Pose2D state_;
};

#endif
