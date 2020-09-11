#include "local_planner.h"

localPlanner::localPlanner()
{
	// Set some default values for anything with a non-zero default
	vision_angle_ = M_PI;
	free_path_length_weight_ = clearance_weight_ = distance_to_goal_weight_ = 1;
}

void localPlanner::trimObstacles(float now)
{
	float upper_angle = state_.theta + 0.5*vision_angle_;
	float lower_angle = state_.theta - 0.5*vision_angle_;

	for (auto obs = ObstacleList.begin(); obs != ObstacleList.end();)	// Clarification: obs is of type std::list<Obstacle>::iterator
	{
		// If an obstacle is too old, get rid of it
		if (now - obs->timestamp > obstacle_memory_) {
			ObstacleList.erase(obs);
			continue;
		}

		// Check to see where in the reference frame obs lies
		float obs_angle = atan2(obs->loc[1], obs->loc[0]);  				// in the range [-pi, pi]

		// If it is within the field of view (i.e. we have new data) erase the old data
		if (obs_angle < upper_angle and obs_angle > lower_angle){
			ObstacleList.erase(obs);
		}
		// Otherwise keep it in memory until it grows stale
		else{
			obs++;
		}
	}
}

void localPlanner::importObstacles(std::vector<Eigen::Vector2f> &obstacles)
{
	float now = ros::Time::now().toSec();
	trimObstacles(now);

	for (const auto &obs : obstacles)
	{
		ObstacleList.push_back(Obstacle {obs, now});
	}
}