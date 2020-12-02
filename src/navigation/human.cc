#include "human.h"
#include "shared/math/math_util.h"
#include "visualization/visualization.h"
#include "shared/math/line2d.h"
#include <iostream>

using Eigen::Vector2f;
using std::vector;
using std::cout;
using std::endl;
using math_util::Sq;
using geometry::line2f;
using visualization::DrawPoint;
using visualization::DrawCross;
using visualization::DrawLine;
using visualization::DrawArc;


namespace human{

// Constructor
Human::Human() :
standing_(false),
FOV_(3*M_PI/4),
vision_range_(3)
{}

// Getters
Vector2f Human::getLoc() 		{return loc_;}
float Human::getAngle()  		{return angle_;}
Vector2f Human::getVel() 		{return vel_;}
float Human::getAngularVel()	{return angular_vel_;}
float Human::getFOV()			{return FOV_;}
bool Human::isStanding()		{return (standing_ or isMoving());}
bool Human::isMoving()			{return vel_.norm() + abs(angular_vel_) > 0.005;}


// Setters
void Human::setLoc(Vector2f loc) 		{loc_   	  = loc;}
void Human::setVel(Vector2f vel)		{vel_   	  = vel;}
void Human::setAngularVel(float omega)	{angular_vel_ = omega;}
void Human::setFOV(float phi)			{FOV_ 		  = phi;}
void Human::setStanding(bool state)		{standing_    = state;}
void Human::setAngle(float theta) {
	angle_ 	    = theta; 
	R_map2local = Eigen::Rotation2Df(-angle_);
	R_local2map = Eigen::Rotation2Df(angle_);
}


// Set Cost Parameters
void Human::setSafetyStdDev(float sigma_x, float sigma_y) {
	safety_x_variance_ = Sq(sigma_x); 
	safety_y_variance_ = Sq(sigma_y);
}

void Human::setVisibilityStdDev(float sigma_x, float sigma_y){
	visibility_x_variance_ = Sq(sigma_x);
	visibility_y_variance_ = Sq(sigma_y);
}

void Human::setHiddenDecay(float k){
	hidden_decay_constant_ = k;
}



// Cost Methods
float Human::safetyCost(Eigen::Vector2f robot_loc) {
	Eigen::Vector2f local_loc = toLocalFrame(robot_loc);
	float cost = exp( -Sq( local_loc.x() )/safety_x_variance_ - Sq( local_loc.y() )/safety_y_variance_ );
	return cost;
}

float Human::visibilityCost(Eigen::Vector2f robot_loc){
	Eigen::Vector2f local_loc = toLocalFrame(robot_loc);
	float cost = 0;

	if (isVisible(local_loc)){
		cost = exp( -Sq( local_loc.x() )/visibility_x_variance_ - Sq( local_loc.y() )/visibility_y_variance_ );
	}

	return cost;
}

// Note that this function does not test whether the robot is hidden
float Human::hiddenCost(Eigen::Vector2f robot_loc, Eigen::Vector2f obs_loc){
	Eigen::Vector2f local_loc = toLocalFrame(robot_loc);
	float cost = 0;

	if (isVisible(local_loc)){
		cost = 1/(hidden_decay_constant_ * (obs_loc - robot_loc).norm());
	}

	return cost;
}



// Visualization
void Human::show(amrl_msgs::VisualizationMsg &msg){
	Vector2f FOV_point1 = vision_range_*Vector2f(cos(FOV_/2), sin(FOV_/2));
	Vector2f FOV_point2 = vision_range_*Vector2f(cos(-FOV_/2), sin(-FOV_/2));

	// Assuming global vis msg
	DrawArc(loc_, 0.5, 0, 2*M_PI, 0x0e07de, msg);						// Human Location
	DrawArc(loc_, 3.0, angle_-FOV_/2, angle_+FOV_/2, 0x000000, msg);	// Field of view
	DrawLine(loc_, toMapFrame(FOV_point1), 0x000000, msg);				// Field of view boundary
	DrawLine(loc_, toMapFrame(FOV_point2), 0x000000, msg);				// Field of view boundary
}



// Utility
Vector2f Human::toLocalFrame(Vector2f p){
	return R_map2local*(p - loc_);
}

Vector2f Human::toMapFrame(Vector2f p){
	return loc_ + R_local2map * p;
}

// Must be in the local frame
bool Human::isVisible(Vector2f local_loc){
	float vision_angle = atan2(local_loc.y(), local_loc.x());
	return(vision_angle > -FOV_/2 and vision_angle < FOV_/2);
}

// Check if the robot is hidden from view (robot_loc is in map frame)
bool Human::isHidden(Vector2f robot_loc, vector_map::VectorMap &map){
	if (map.Intersects(loc_, robot_loc)) return true;
	return false;
}

// Update the human location according to it's velocity
void Human::move(float dt){
	if (dt < 0.01) return;
	setLoc(loc_ + vel_*dt);
	float new_angle = angle_ + angular_vel_*dt;

	// Keep angle in the range (-pi,pi)
	if (new_angle >= M_PI){
		new_angle -= 2*M_PI;
	}else if(new_angle < -M_PI){
		new_angle += 2*M_PI;
	}

	setAngle(new_angle);
}


} // end namespace human
