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
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "stdio.h" // Mark added

#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2f;

using namespace math_util;
using namespace ros_helpers;

// Note: namespace allows for "global" variables that exist only in this file
namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// Delta t of the control loop
float dt_ = 1/20.0;	// made not constant to adapt for variance/lag
// NOTE: This may cause issues with the latency compensator

// Robot Parameters
const float observation_delay_ 	= 0.0;
const float actuation_delay_ 	= 0.0;
const float vision_angle_ 		= 3*M_PI/2;  
const float vision_range_ 		= 10; 		// based on sim, grid squares are 2m
const float curvature_max_ 		= 1/1.0;	// can take turns as tight as 1m
const float clearance_limit_	= 1.0;

// Fake Robot Parameters
const float car_width_ 	= 0.27;		// width
const float car_length_ = 0.5;		// length
const float padding_ 	= 0.1;		// padding
const float wheelbase_ 	= 0.324;	// wheelbase
const Vector2f pmin(0, car_width_/2+padding_);
const Vector2f pdif((wheelbase_+car_length_)/2 + padding_,  car_width_/2+padding_);
const Vector2f pmax((wheelbase_+car_length_)/2 + padding_, -car_width_/2-padding_);

// Robot Limits
const float max_vel_   =  1.0;
const float min_vel_   = -1.0;
const float max_accel_ =  4.0;
const float min_accel_ = -4.0;

// Global one-time variables
Eigen::Vector2f local_goal_vector_;
bool navigation_success_ = false;
bool init_ = true;
bool init_vel_ = true;
ros::Time time_prev_;
} //namespace

namespace navigation {

bool isBetween(const Vector2f p0, const Vector2f p1, const Vector2f p2, const Vector2f p)
{
	/* Determine if p is inside the cone defined by p0, p1, and p2 
	    p2
	   /
	  /
	p0
	  \      p
	   \
	    p1
	*/

	float angle_p1 = atan2(p1[1] - p0[1], p1[0] - p0[0]);
	float angle_p2 = atan2(p2[1] - p0[1], p2[0] - p0[0]);
	float angle_p  = atan2( p[1] - p0[1],  p[0] - p0[0]);

	if (angle_p1 > 0 and angle_p2 < 0){
		return (angle_p > angle_p1 or angle_p < angle_p2);
	}
	else{
		return (angle_p > angle_p1 and angle_p < angle_p2);
	}
}

float getAngleBetween(const Vector2f point_A, const Vector2f point_B, const Vector2f point_C){
	/* returns absolute value of angle between AC and BC
	        C
				/   \
		   / thC \
		  a       b
		 /         \
		/           \
	 B______c______A
	*/
	Vector2f vec1 = point_B - point_C;
	Vector2f vec2 = point_A - point_C;
	float angle_C = acos((vec1/vec1.norm()).dot(vec2/vec2.norm()));
	angle_C = angle_C + M_PI * (angle_C < 0); // negative indicates obtuse angle
	return angle_C;
}

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
		robot_loc_(0, 0),
		robot_angle_(0),
		robot_vel_(0, 0),
		robot_omega_(0),
		LC_(actuation_delay_, observation_delay_, dt_),
		nav_complete_(true),
		nav_goal_loc_(0, 0),
		// nav_goal_angle_(0),
		free_path_length_weight_(1),
		clearance_weight_(1),
		distance_to_goal_weight_(1),
		obstacle_memory_(1)
{
	drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
	viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
	local_viz_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_local");
	global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
	InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
	nav_goal_loc_ = loc;
	// nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
	robot_loc_ = loc;
	robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc, float angle,
								const Vector2f& vel, float ang_vel) {
	LC_.recordObservation(loc[0], loc[1], angle);
	state2D current_state = LC_.predictedState();

	odom_loc_ 	 = {current_state.x, current_state.y};
	odom_angle_  =  current_state.theta;
	robot_vel_ 	 = {current_state.vx, current_state.vy};
	robot_omega_ =  current_state.omega;

	R_odom2base_ << cos(odom_angle_), -sin(odom_angle_),
				    sin(odom_angle_),  cos(odom_angle_);

	init_ = false;
}

void Navigation::trimObstacles(double now)
{
	float upper_angle = odom_angle_ + 0.49*vision_angle_;
	float lower_angle = odom_angle_ - 0.49*vision_angle_;

	Vector2f lower_lim_point = BaseLink2Odom({cos(lower_angle), sin(lower_angle)});
	Vector2f upper_lim_point = BaseLink2Odom({cos(upper_angle), sin(upper_angle)});

	for (auto obs = ObstacleList_.begin(); obs != ObstacleList_.end();)	// Clarification: obs is of type std::list<Obstacle>::iterator
	{
		// If an obstacle is too old, get rid of it
		if (now - obs->timestamp > obstacle_memory_) {
			obs = ObstacleList_.erase(obs);
			continue;
		}
		
		// If it is within the field of view (i.e. we have new data) erase the old data
		if (isBetween(odom_loc_, lower_lim_point, upper_lim_point, obs->loc)){
			obs = ObstacleList_.erase(obs);
		}
		// Otherwise keep it in memory until it grows stale
		else{
			obs++;
		}
	}
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
	trimObstacles(time);

	for (auto &obs_loc : cloud)
	{
		ObstacleList_.push_back(Obstacle {BaseLink2Odom(obs_loc), time});
	}
}

void Navigation::showObstacles()
{
	int i = 0;
	int cutoff_count = ObstacleList_.size()/1000;
	for (const auto &obs : ObstacleList_)
	{
		if (i != cutoff_count) {i++; continue;} // ensure that no more than 1000 obstacles are displayed
		visualization::DrawCross(Odom2BaseLink(obs.loc), 0.05, 0x000000, local_viz_msg_);
		i = 0;
	}
}

void Navigation::createPossiblePaths(float num)
{
	PossiblePaths_.clear();

	float curve_increment = 2*curvature_max_/num;
	for (int i = 0; i < num; i++)
	{
		float curvature = -curvature_max_ + i*curve_increment;
		// Enforce max radius of 1km (any bigger and the angles get so small the math is bad)
		if (std::abs(curvature) < 0.001) curvature = 0.001;
		PossiblePaths_.push_back(PathOption {curvature, // curvature
											 0,			// clearance
											 0,			// free path length
											 0,			// distance to goal
											 0,			// cost
											 {0,0},		// obstruction location
											 {0,0},		// closest point location
											 {0,0}});	// end point of the movement
	}
}

void Navigation::trimPathLength(PathOption &path, Vector2f goal)
{
	// NOTE: Defined in the Base Link frame
	Vector2f P_center = {0, 1/path.curvature};	// Center of rotation

	float angle = getAngleBetween(goal, {0,0}, P_center);
	if (goal[0] < 0) angle = 2*M_PI - angle; 	// negative x means angle > 180°

	// If trimmed path length is less than free path length, substitute in the trimmed value
	if (abs(angle/path.curvature) < path.free_path_length)
	{
		path.free_path_length = abs(angle/path.curvature);
		path.obstruction = P_center + 1/path.curvature * (goal - P_center)/(goal - P_center).norm();
	} 
}

// Calculate free path length for a given path
void Navigation::predictCollisions(PathOption& path){
	float radius = 1/path.curvature; // can be negative

	Vector2f turning_center(0,radius); // point of rotation
	
	// Get radii to specific points based on curvature
	float rmin = (Sign(radius)*turning_center - pmin).norm();	// radius from center of rotation to innermost point on the rear axle
	float rdif = (Sign(radius)*turning_center - pdif).norm();	// radius from center of rotation to the turning corner of the robot
	float rmax = (Sign(radius)*turning_center - pmax).norm();	// radius from center of rotation to the outermost point on the robot

	// Default obstruction point is full simecircle of rotation
	float fpl_min = abs(M_PI*radius);
	Vector2f p_obstruction(0, 2*radius);

	// Iterate through points in point cloud
	for (const auto &obs : ObstacleList_)
	{
		Vector2f obs_loc = Odom2BaseLink(obs.loc); 				// put obstacle location in base_link frame
		float obs_radius = (turning_center - obs_loc).norm();	// distance to obstacle from turning center

		// Ignore point if it's further away than the most direct path to current goal (approximately)
		if (obs_loc.norm() > local_goal_vector_[0] + car_length_) continue;

		// Check if point will obstruct car
		if (obs_radius > rmin && obs_radius < rmax) {
			Vector2f p_current;
			
			// Inner side collision
			if (obs_radius < rdif) {
				float phi = acos(( Sign(radius)*radius-car_width_/2-padding_)/obs_radius );
				float x = obs_radius*sin(phi);
				p_current = {x, Sign(radius)*(car_width_/2+padding_)};
			
			// Front collision
			}else if (obs_radius > rdif) {
				float phi = asin(((wheelbase_+car_length_)/2+padding_)/obs_radius);
				float y = radius - Sign(radius)*obs_radius*cos(phi);
				p_current = {(wheelbase_+car_length_)/2+padding_, y};
			}
			
			// Use law of cosines to compute free path length
			float side_length = (obs_loc-p_current).norm();
			float theta = acos((side_length*side_length-2*obs_radius*obs_radius)/(-2*obs_radius*obs_radius));
			
			// Solve for free path length of the current point
			float fpl_current = Sign(radius)*theta*radius;

			// If this is the first loop or this is the smallest fpl so far, record this value
			if (fpl_current < fpl_min){
				fpl_min = fpl_current;
				p_obstruction = obs_loc;
			}
		}
	}

	// Save results to path struct
	path.obstruction = p_obstruction;
	path.free_path_length = fpl_min;
}

// New base_link frame clearance calc
void Navigation::calculateClearance(PathOption &path){
	// Warning: These can be negative
	float radius = 1/path.curvature;
	float theta = path.free_path_length/radius;

	// Look 3 car lengths ahead
	float look_ahead_dist = 3*car_length_;

	// Get start and end points
	Vector2f center(0,radius);
	Vector2f start_point(0,0);
	Vector2f end_point(0,0);
	end_point.x() = radius*sin(theta) + look_ahead_dist*cos(theta);
	end_point.y() = radius*(1-cos(theta)) + look_ahead_dist*sin(theta);

	// Flip points for isBetween turning direction dependency
	Vector2f point_1 = (radius > 0 ? start_point : end_point);
	Vector2f point_2 = (radius > 0 ? end_point : start_point);

	// Initialize clearance at its maximum allowed value
	float min_clearance = clearance_limit_;

	// Get angle between start and end
	Vector2f closest_point(0,0);
	for (const auto &obs : ObstacleList_)
	{
		Vector2f obs_point = Odom2BaseLink(obs.loc);

		if (isBetween(center, point_1, point_2, obs_point))
		{
			float radius_to_point = (center-obs_point).norm();
			float clearance = abs(radius_to_point - abs(radius));
			if (clearance < min_clearance)
			{
				min_clearance = clearance;
				closest_point = obs_point;
			}
		}
	}
	path.clearance = min_clearance;
	path.closest_point = closest_point;
}

void Navigation::setLocalPlannerWeights(float w_FPL, float w_C, float w_DTG)
{
	free_path_length_weight_ = w_FPL;
	clearance_weight_ 		 = w_C;
	distance_to_goal_weight_ = w_DTG;
}

PathOption Navigation::getGreedyPath(Vector2f goal_loc)
{
	int num_paths = 20;

	// Clear out possible paths and reinitialize
	createPossiblePaths(num_paths);

	// Initialize output and cost
	PathOption BestPath;
	float min_cost = 1e10;

	// Vectors to store results
	vector<double> free_path_length_vec;
	vector<double> clearance_padded_vec;
	vector<double> distance_to_goal_vec;

	// Get best parameter from all possible paths for normalization
	float max_free_path_length = 1e-5;
	float max_clearance_padded = 1e-5;
	float min_distance_to_goal = 1e5;

	for (auto &path : PossiblePaths_)
	{
		// Update FLP, Clearance, Closest Point, Obstruction, End Point
		predictCollisions(path);
		trimPathLength(path, goal_loc);
		calculateClearance(path);
		float clearance_padded = path.clearance - (car_width_/2+padding_*2);
		if (clearance_padded < 0) clearance_padded = 1e-5;

		path.distance_to_goal = (path.obstruction - goal_loc).norm(); // approximately

		max_free_path_length = std::max(path.free_path_length, max_free_path_length);
		max_clearance_padded = std::max(clearance_padded, max_clearance_padded);
		min_distance_to_goal = std::min(path.distance_to_goal, min_distance_to_goal);

		free_path_length_vec.push_back(path.free_path_length);
		clearance_padded_vec.push_back(clearance_padded);
		distance_to_goal_vec.push_back(path.distance_to_goal);
	}

	// Iterate through paths to find the best one
	for (int i = 0; i < num_paths; i++)
	{
		float free_path_length = free_path_length_vec.at(i);
		float clearance_padded = clearance_padded_vec.at(i);
		float distance_to_goal = distance_to_goal_vec.at(i);

		// Decrease cost with larger free path length
		float free_path_length_cost = -(free_path_length/max_free_path_length) * free_path_length_weight_;
		// Increase cost with more 1/clearance (normalized and padded)
		float clearance_padded_cost	=  (max_clearance_padded/clearance_padded) * clearance_weight_;
		// Increase cost with larger distance to goal
		float distance_to_goal_cost =  (distance_to_goal/min_distance_to_goal) * distance_to_goal_weight_;

		float cost = free_path_length_cost + clearance_padded_cost + distance_to_goal_cost;
		if (cost < min_cost) {
			min_cost = cost;
			BestPath = PossiblePaths_.at(i);
			BestPath.cost = cost;
		}
	}
	return BestPath;
}

// Limit Velocity to follow both acceleration and velocity limits
float Navigation::limitVelocity(float vel) {
	// For some very strange reason, the y-term of velocity is initialized at infinity...
	float current_speed = robot_vel_.norm();
	if (init_vel_) { current_speed = 0; init_vel_ = false; }
	ros::Time time_now = ros::Time::now();
	ros::Duration delta_time = time_now - time_prev_;
	dt_ = delta_time.toSec();
	// Changed to account for 2d speed
	float new_vel = std::min({vel, 		current_speed + max_accel_ * dt_, max_vel_});
	float cmd_vel = std::max({new_vel, 	current_speed + min_accel_ * dt_, min_vel_});
	time_prev_ = time_now;
	return cmd_vel;
}

void Navigation::moveAlongPath(PathOption path){
	float current_speed = robot_vel_.norm();
	float decel_dist = 0.1+-0.5*current_speed*current_speed/min_accel_;
	float cmd_vel = (path.free_path_length > decel_dist) ? max_vel_ : 0.0;
	// if (navigation_success_) cmd_vel = 0;
	driveCar(path.curvature, limitVelocity(cmd_vel));
}

void Navigation::plotPathDetails(PathOption path){
	// Outline car
	visualization::DrawLine({-(car_length_-wheelbase_)/2, car_width_/2},  {(wheelbase_+car_length_)/2, car_width_/2},  0x000000, local_viz_msg_);	//top
	visualization::DrawLine({-(car_length_-wheelbase_)/2, -car_width_/2}, {(wheelbase_+car_length_)/2, -car_width_/2}, 0x000000, local_viz_msg_);	//bottom
	visualization::DrawLine({(wheelbase_+car_length_)/2, car_width_/2},   {(wheelbase_+car_length_)/2, -car_width_/2}, 0x000000, local_viz_msg_);	//front
	visualization::DrawLine({-(car_length_-wheelbase_)/2, -car_width_/2}, {-(car_length_-wheelbase_)/2, car_width_/2}, 0x000000, local_viz_msg_);	//back

	// Place green cross at obstruction point if it exists
	if (path.obstruction.norm() > 0)
		visualization::DrawCross(path.obstruction, 0.5, 0x00ff00, local_viz_msg_);

	// Draw all the possible path options
	for (auto other_path : PossiblePaths_)
	{
		visualization::DrawPathOption(other_path.curvature, other_path.free_path_length, 0.0 /*clearance*/, local_viz_msg_);
	}

	// Draw path option
	visualization::DrawPathOption(path.curvature, path.free_path_length, path.clearance, local_viz_msg_);
	
	// Place red cross at goal position
	visualization::DrawCross(Odom2BaseLink(local_goal_vector_), 0.5, 0xff0000, local_viz_msg_);

	// Draw Closest Point for Clearance
	visualization::DrawCross(path.closest_point, 0.25, 0xf07807, local_viz_msg_);
}

void Navigation::printPathDetails(PathOption path){
	std::cout << "loop time:        " << dt_ << std::endl
			  << "current velocity: " << robot_vel_.norm() << std::endl
			  << "curvature:        " << path.curvature << std::endl
			  << "clearance:        " << path.clearance << std::endl
			  << "free path length: " << path.free_path_length << std::endl
			  << "distance_to_goal: " << path.distance_to_goal << std::endl
			  << "cost:             " << path.cost << std::endl;
	printVector(path.obstruction, "obstruction: ");
	printVector(path.closest_point, "closest point: ");
	printVector(path.end_point, "end point: ");
	printVector(local_goal_vector_, "goal position");
	std::cout << " - - - - - - - - - - - - - - - " << std::endl;
}

void Navigation::printVector(Vector2f print_vector, string vector_name){
	std::cout << vector_name << "\t x= " << print_vector.x() << ", y= " << print_vector.y() << std::endl;
}

void Navigation::driveCar(float curvature, float velocity){
	drive_msg_.header.seq++;
	drive_msg_.header.stamp = ros::Time::now();
	drive_msg_.curvature = curvature;
	drive_msg_.velocity = velocity;
	drive_pub_.publish(drive_msg_);

	// Record input in the latency compensator
	geometry_msgs::Twist cartesian_velocity = AckermannIK(curvature, velocity);
	LC_.recordNewInput(cartesian_velocity.linear.x, 
					   cartesian_velocity.linear.y, 
					   cartesian_velocity.angular.z);
}

// Ackerman Forward/Inverse Kinematics
// FK never gets used
AckermannCurvatureDriveMsg Navigation::AckermannFK(float x_dot, float y_dot, float omega){
	float theta = atan2(y_dot, x_dot);
	float velocity = x_dot*cos(theta);
	float curvature = omega/velocity;

	AckermannCurvatureDriveMsg ackermann_msg;
	ackermann_msg.velocity = velocity;
	ackermann_msg.curvature = curvature;
	return ackermann_msg;
}
geometry_msgs::Twist Navigation::AckermannIK(float curvature, float velocity){
	float theta = odom_angle_;
	float x_dot = velocity*cos(theta);
	float y_dot = velocity*sin(theta);
	float omega = velocity*curvature;

	geometry_msgs::Twist vel;
	vel.linear.x = x_dot;
	vel.linear.y = y_dot;
	vel.angular.z = omega;
	return vel;
}

// Frame Transformations
Eigen::Vector2f Navigation::BaseLink2Odom(Eigen::Vector2f p) {return odom_loc_ + R_odom2base_*p;}
Eigen::Vector2f Navigation::Odom2BaseLink(Eigen::Vector2f p) {return R_odom2base_.transpose()*(p - odom_loc_);}



//========================= GLOBAL PLANNER - NODE FUNCTIONS ============================//

// Done: Alex
string Navigation::getNewID(int xi, int yi){
	string id = std::to_string(xi) + "_" +  std::to_string(yi);
	return id;	
}

// Done: Alex
float Navigation::edgeCost(const Node &node_A, const Node &node_B){
	// Basic distance cost function
	return((node_A.loc - node_B.loc).norm());
}

// Helper Function (untested)
// outputs 2 lines parallel to edge that are displaced by a given offset
std::array<line2f,2> getCushionLines(line2f edge, float offset){
	Vector2f normal_vec = edge.UnitNormal();
	Vector2f cushion_A_point_1 = edge.p0 + normal_vec * offset;
	Vector2f cushion_A_point_2 = edge.p1 + normal_vec * offset;
	Vector2f cushion_B_point_1 = edge.p0 - normal_vec * offset;
	Vector2f cushion_B_point_2 = edge.p1 - normal_vec * offset;
	return {line2f(cushion_A_point_1, cushion_A_point_2), line2f(cushion_B_point_1, cushion_B_point_2)};
}

// TODO: Alex
bool Navigation::isValidNeighbor(const Node &node, const Neighbor &neighbor){
	// Check for adjacency
	int x_offset = node.index.x() - neighbor.node_index.x();
	int y_offset = node.index.y() - neighbor.node_index.y();
	if (not (abs(x_offset) == 1 or abs(y_offset) == 1)) // changed so that node can't be neighbors with itself
		return false;

	// Create 3 lines: 1 from A to B and then the others offset from that as a cushion
	Vector2f offset(map_resolution_ * x_offset, map_resolution_ * y_offset);
	Vector2f neighbor_loc = node.loc + offset;
	const line2f edge(node.loc, neighbor_loc);
	auto cushion_lines = getCushionLines(edge, 0.5);
	cushion_lines[0].p0.x()++;	// just put this line here so it would compile

	// TODO: Load the map file somewhere in the planner

	// Check for collisions
	// for (const line2f map_line : map_.lines)
 //    {
	// 	bool intersects_main = map_line.Intersects(node_line);
	// 	bool intersects_cushion_0 = map_line.Intersects(cushion_lines[0]);
	// 	bool intersects_cushion_1 = map_line.Intersects(cushion_lines[1]);
	// 	if (intersects_main or intersects_cushion_0 or intersects_cushion_1)
	// 		return false;
 //    }

	return true;
}

// Done: Alex (untested)
vector<Neighbor> Navigation::getNeighbors(const Node &node){
	vector<Neighbor> neighbors;

	int xi = node.index.x();
	int yi = node.index.y();

	float diagonal_path_length = sqrt(2)*map_resolution_;	// Assuming a straight line path
	float straight_path_length  = map_resolution_;			// Straight line distance

	// Note the each neighbor has the form {ID, curvature, path length, index}
	neighbors.push_back({Vector2i(xi-1, yi+1), getNewID(xi-1, yi+1), diagonal_path_length, 0});	// Left and up
	neighbors.push_back({Vector2i(xi  , yi+1), getNewID(xi,   yi+1), straight_path_length, 1});	// Directly up
	neighbors.push_back({Vector2i(xi+1, yi+1), getNewID(xi+1, yi+1), diagonal_path_length, 2});	// Right and up
	neighbors.push_back({Vector2i(xi-1, yi  ), getNewID(xi-1, yi  ), straight_path_length, 3});	// Directly left
	neighbors.push_back({Vector2i(xi+1, yi  ), getNewID(xi+1, yi  ), straight_path_length, 5});	// Directly right
	neighbors.push_back({Vector2i(xi-1, yi-1), getNewID(xi-1, yi-1), diagonal_path_length, 6}); // Left and down
	neighbors.push_back({Vector2i(xi  , yi-1), getNewID(xi,   yi-1), straight_path_length, 7});	// Directly down
	neighbors.push_back({Vector2i(xi+1, yi-1), getNewID(xi+1, yi-1), diagonal_path_length, 8});	// Right and down

	return neighbors;
}

// Done: Alex (untested)
Node Navigation::newNode(const Node &old_node, int neighbor_index){
	// Change in index, not in position
	int dx = 0; 
	int dy = 0;
	Node new_node;
	
	// Untested
	dx = (neighbor_index % 3 == 2) - (neighbor_index % 3 == 0);
	dy = (neighbor_index < 3) - (neighbor_index > 5);

	new_node.loc 	   = old_node.loc + map_resolution_ * Vector2f(dx, dy);
	new_node.index 	   = old_node.index + Eigen::Vector2i(dx, dy);
	new_node.cost 	   = old_node.cost + edgeCost(old_node, new_node);
	new_node.parent    = old_node.key;
	new_node.key 	   = getNewID(new_node.index.x(), new_node.index.y());
	new_node.neighbors = getNeighbors(new_node);

	return new_node;
}

// Done: Alex (untested)
void Navigation::initializeMap(Eigen::Vector2f loc, float resolution){
	map_resolution_ = resolution;
	nav_map_.clear();

	int xi = loc.x()/map_resolution_;
	int yi = loc.y()/map_resolution_;

	Node start_node;
	start_node.loc 	  = loc;
	start_node.index  = Eigen::Vector2i(xi, yi);
	start_node.cost   = 0;
	start_node.parent = "START";
	start_node.key    = "START";
	start_node.neighbors = getNeighbors(start_node);

	nav_map_[start_node.key] = start_node;
}

// TODO: Connor
void Navigation::plotNodeNeighbors(const Node &node){
	// Visualize the node and it's immediate neighbors using global_viz_msg_ (defined in line 52)
	// Make the node a small cross
	// Plot the paths between the node and each neighbors (straight lines)
	// The nodes are arranged and labelled like this:

	// 0---1---2     ^ y
	// |   |   |     |
	// 3---4---5     o---> x
	// |   |   |
	// 6---7---8

	// You will need to make use of the new function I wrote: newNode(const Node &old_node, int neighbor_index) {untested, sorry}
	// The straight line distance between nodes is contained in the global variable map_resolution_;
	// I've already put the necessary code in to make it run in the main program so you just need to work here

	// Here is the syntax for the needed visualization functions:
	// visualization::DrawPoint(const Vector2f& p, uint32_t color, VisualizationMsg& msg)
	// visualization::DrawLine(const Vector2f& p0, const Vector2f& p1, uint32_t color, VisualizationMsg& msg)
	// visualization::DrawCross(const Eigen::Vector2f& location, float size, uint32_t color, VisualizationMsg& msg)
	// visualization::DrawArc(const Vector2f& center, float radius, float start_angle, float end_angle, uint32_t color, VisualizationMsg& msg)

	// And here are some nice options for the uint32_t color inputs in those functions above:
	// Blue: 0x000dff
	// Red: 0xff0000
	// Green: 0x009c08
	// Orange: 0xff9900
	// Black: 0x000000

	// Here's some starter code/API example for the node stuff
	for (size_t i = 0; i < node.neighbors.size(); i++){
		// Get the ID for this neighboring node
		string neighbor_id = node.neighbors[i].key;
		// Check if node already exists
		if (not nav_map_.count(neighbor_id)){
			// If it does not, create a new one in the nav_map_
			Node neighbor = newNode(node, i);
		}

		// Do the plot stuff here
		
	}
}

//========================= GLOBAL PLANNER - PATH PLANNING ============================//
vector<string> Navigation::getGlobalPath(){
	frontier_.Push("START", 0.0);

	int loop_counter = 0; // exit condition if while loop gets stuck (goal unreachable)
	while(!frontier_.Empty() && loop_counter < 1000)
	{
		string current_key = frontier_.Pop();
		Node current_node = nav_map_[current_key];
		
		// Are we there yet?
		if ( (current_node.loc - nav_goal_loc_).norm() < map_resolution_/2 )
		{
			navigation_success_ = true;
			break;
		}

		for(auto &next_neighbor : current_node.neighbors)
		{
			// Skip if not valid (intersects with map)
			float new_cost = current_node.cost + next_neighbor.path_length;
			cout << new_cost << endl;

			bool uncharted = (nav_map_.count(next_neighbor.key) == 0); // change to count()
			if (uncharted){
				//make NewNode out of neighbor
			}
			
			// first condition on this if statement: If a map can't find() an element corresponding to
		  // the key, it returns the iterator corresponding to its end(), so you can compare to that
			if (uncharted or (new_cost < nav_map_[next_neighbor.key].cost))
			{
				//A*
			}
		}
		loop_counter++;
	}
	// look at frontier, get viable neighbors (not in collision)
	// for each neighbor, get neighbor_cost
	// if neighbor has never been visited OR neighbor_cost is lower than that node's cost:
	//   set that node's cost to neighbor_cost
	//   compute priority = neighbor_cost + heuristic(goal, neighbor)
	//   add that to frontier by order of priority
	vector<string> global_path;
	global_path.push_back("START");
	return global_path;
}

// Main Loop
void Navigation::Run() {
	visualization::ClearVisualizationMsg(local_viz_msg_);
	visualization::ClearVisualizationMsg(global_viz_msg_);

	// Added this section for anything that we only want to happen once (like setup function)
	// NOTE: This init function is added to only initialize AFTER odometry has been received
	if  (init_){
		while (init_ and ros::ok()){
			ros::spinOnce();
			ros::Rate(10).sleep();
		}
		cout << "hey!" << endl;
		local_goal_vector_ = Vector2f(4,0); //carrot on a stick 4m ahead, will eventually be a fxn along global path
		time_prev_ = ros::Time::now();
		nav_goal_loc_ = Vector2f(20,-10); //random
		// Initialize blueprint map
		map_.Load("maps/GDC1.txt");
		cout << "Initialized GDC1 map with " << map_.lines.size() << " lines." << endl;
		// Node Visualization Testing
		initializeMap({0,0}, map_resolution_);  // (location, resolution)
	}
	plotNodeNeighbors(nav_map_["START"]);

	showObstacles();
	PathOption BestPath = getGreedyPath(local_goal_vector_);
	moveAlongPath(BestPath);
	// printPathDetails(BestPath);
	plotPathDetails(BestPath);

	// If we have reached our goal we can stop (not relevant for dynamic goal)
	float dist_to_goal = (odom_loc_-local_goal_vector_).norm();
	float current_speed = robot_vel_.norm();
	if (current_speed > 2.0) current_speed = 0; // disregards initial infinite velocity
	float stopping_dist = 0.1+-0.5*current_speed*current_speed/min_accel_;
	if (dist_to_goal <= stopping_dist){
		navigation_success_ = true;
		ROS_WARN("Navigation success!");
	}

	viz_pub_.publish(local_viz_msg_);
	viz_pub_.publish(global_viz_msg_);
}

}  // namespace navigation
