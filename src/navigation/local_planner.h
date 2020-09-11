#ifndef LOCAL_PLANNER_HH
#define LOCAL_PLANNER_HH

#include <vector>
#include <list>
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
	float timestamp;
};

class localPlanner{
public:
	// Constructor
	localPlanner();

	// Setters/Getters
	void setState(float x, float y, float theta);
	geometry_msgs::Pose2D getState();

	void setObstacleMemory(float t);
	float getObstacleMemory();

	void setVisionAngle(float theta);
	float getVisionAngle();

	void setPlannerWeights(float w_FPL, float w_C, float w_DTG);
	std::vector<float> getPlannerWeights();

	// Public API
	void importObstacles(std::vector<Eigen::Vector2f> &obstacles);
	PathOption getGreedyPath();

private:
	// Called by importOstacles
	void trimObstacles(float timestamp);

	// Called by getGreedyPlannerPath
	void createPossiblePaths();
	void predictCollisions();
	void showPaths() const;

	// Private Members
	std::list<Obstacle> ObstacleList;
	std::vector<PathOption> PossiblePaths;

	float free_path_length_weight_;
	float clearance_weight_;
	float distance_to_goal_weight_;

	geometry_msgs::Pose2D state_;

	float obstacle_memory_; 	// Time to keep old obstacles in seconds
	float vision_angle_;		// The observation angle of the LiDAR in radians (assumed to be symmetrical about the x-axis)
};

#endif
