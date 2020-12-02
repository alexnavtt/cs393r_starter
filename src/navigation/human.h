#ifndef HUMAN_SIM_HH
#define HUMAN_SIM_HH

#include <vector>
#include "eigen3/Eigen/Dense"

#include "amrl_msgs/VisualizationMsg.h"
#include "vector_map/vector_map.h"

namespace human{

class Human{
public:
	Human();

	// Getters
	Eigen::Vector2f getLoc();
	float getAngle();
	Eigen::Vector2f getVel();
	float getAngularVel();
	float getFOV();
	bool isStanding();
	bool isMoving();

	// Setters
	void setLoc(Eigen::Vector2f human_loc);
	void setAngle(float theta);
	void setVel(Eigen::Vector2f human_vel);
	void setAngularVel(float omega);
	void setFOV(float phi);
	void setStanding(bool state);

	// Set Cost Parameters
	void setSafetyStdDev(float sigma_x, float sigma_y);
	void setVisibilityStdDev(float sigma_x, float sigma_y);
	void setHiddenDecay(float k); // in the form cost = 1/(k*x)

	// Cost Methods (Input points are in map frame)
	float safetyCost(Eigen::Vector2f robot_loc);
	float visibilityCost(Eigen::Vector2f robot_loc);
	float hiddenCost(Eigen::Vector2f robot_loc, Eigen::Vector2f obs_loc);

	// Utility
	bool isHidden(Eigen::Vector2f robot_loc, vector_map::VectorMap &map);
	void move(float dt);

	// Visualization
	void show(amrl_msgs::VisualizationMsg &msg);

private:
	// Human State
	Eigen::Vector2f loc_;
	float angle_;
	Eigen::Vector2f vel_;
	float angular_vel_;

	// Default state is sitting
	bool standing_;

	// Cost Fields
	float FOV_;
	float vision_range_;
	float safety_x_variance_;
	float safety_y_variance_;
	float visibility_x_variance_;
	float visibility_y_variance_;
	float hidden_decay_constant_;
	bool isVisible(Eigen::Vector2f local_loc);

	// Frame Transforms
	Eigen::Matrix2f R_map2local;
	Eigen::Matrix2f R_local2map;
	Eigen::Vector2f toLocalFrame(Eigen::Vector2f);
	Eigen::Vector2f toMapFrame(Eigen::Vector2f);
};

} // end namespace human

#endif
