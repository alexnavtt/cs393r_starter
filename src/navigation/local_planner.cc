#include "local_planner.h"

#include "shared/math/math_util.h"

using std::vector;
using std::list;
using Eigen::Vector2f;
using std::cout;
using std::endl;
using std::string;

using namespace math_util;

//====================== HELPER FUNCTIONS ============================//

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

void printVector(Vector2f print_vector, string vector_name){
	std::cout << vector_name << "\t x= " << print_vector.x() << ", y= " << print_vector.y() << std::endl;
}


//========================== CLASS FUNCTIONS ===============================//

namespace navigation {

LocalPlanner::LocalPlanner() :
	car_width_(0.27),
	car_length_(0.5),
	padding_(0.1),
	wheelbase_(0.324),
	observation_delay_(0.0),
	actuation_delay_(0.0),
	vision_angle_(3*M_PI/2),  
	vision_range_(10), 		// based on sim, grid squares are 2m
	curvature_max_(1/1.0),	// can take turns as tight as 1m
	clearance_limit_(1.0)
{
	pmin_ = Vector2f(0, car_width_/2+padding_);
	pdif_ = Vector2f((wheelbase_+car_length_)/2 + padding_,  car_width_/2+padding_);
	pmax_ = Vector2f((wheelbase_+car_length_)/2 + padding_, -car_width_/2-padding_);
	// Set some default values for anything with a non-zero default
}

void LocalPlanner::createPossiblePaths(int num)
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

void LocalPlanner::trimPathLength(PathOption &path, Vector2f goal)
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
void LocalPlanner::predictCollisions(PathOption& path, const Vector2f goal_loc, const list<Obstacle> &obstacle_list){
	float radius = 1/path.curvature; // can be negative

	Vector2f turning_center(0,radius); // point of rotation
	
	// Get radii to specific points based on curvature
	float rmin = (Sign(radius)*turning_center - pmin_).norm();	// radius from center of rotation to innermost point on the rear axle
	float rdif = (Sign(radius)*turning_center - pdif_).norm();	// radius from center of rotation to the turning corner of the robot
	float rmax = (Sign(radius)*turning_center - pmax_).norm();	// radius from center of rotation to the outermost point on the robot

	// Default obstruction point is full simecircle of rotation
	float fpl_min = abs(M_PI*radius);
	Vector2f p_obstruction(0, 2*radius);

	// Iterate through points in point cloud
	for (const auto &obs : obstacle_list)
	{
		Vector2f obs_loc = obs.loc; 				// put obstacle location in base_link frame
		float obs_radius = (turning_center - obs_loc).norm();	// distance to obstacle from turning center

		// Ignore point if it's further away than the most direct path to current goal (approximately)
		if (obs_loc.norm() > goal_loc[0] + car_length_) continue;

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
void LocalPlanner::calculateClearance(PathOption &path, const list<Obstacle> &obstacle_list){
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
	for (const auto &obs : obstacle_list)
	{
		Vector2f obs_point = obs.loc;

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

void LocalPlanner::setWeights(float w_FPL, float w_C, float w_DTG)
{
	free_path_length_weight_ = w_FPL;
	clearance_weight_ 		 = w_C;
	distance_to_goal_weight_ = w_DTG;
}

PathOption LocalPlanner::getGreedyPath(Vector2f goal_loc, const std::list<Obstacle> &obstacle_list)
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
		predictCollisions(path, goal_loc, obstacle_list);
		trimPathLength(path, goal_loc);
		calculateClearance(path, obstacle_list);
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

void LocalPlanner::plotPathDetails(PathOption path, Vector2f goal_loc, amrl_msgs::VisualizationMsg &msg){
	// Outline car
	visualization::DrawLine({-(car_length_-wheelbase_)/2, car_width_/2},  {(wheelbase_+car_length_)/2, car_width_/2},  0x000000, msg);	//top
	visualization::DrawLine({-(car_length_-wheelbase_)/2, -car_width_/2}, {(wheelbase_+car_length_)/2, -car_width_/2}, 0x000000, msg);	//bottom
	visualization::DrawLine({(wheelbase_+car_length_)/2, car_width_/2},   {(wheelbase_+car_length_)/2, -car_width_/2}, 0x000000, msg);	//front
	visualization::DrawLine({-(car_length_-wheelbase_)/2, -car_width_/2}, {-(car_length_-wheelbase_)/2, car_width_/2}, 0x000000, msg);	//back

	// Place green cross at obstruction point if it exists
	if (path.obstruction.norm() > 0)
		visualization::DrawCross(path.obstruction, 0.5, 0x00ff00, msg);

	// Draw all the possible path options
	for (auto other_path : PossiblePaths_)
	{
		visualization::DrawPathOption(other_path.curvature, other_path.free_path_length, 0.0 /*clearance*/, msg);
	}

	// Draw path option
	visualization::DrawPathOption(path.curvature, path.free_path_length, path.clearance, msg);
	
	// Place red cross at goal position
	visualization::DrawCross(goal_loc, 0.5, 0xff0000, msg);

	// Draw Closest Point for Clearance
	visualization::DrawCross(path.closest_point, 0.25, 0xf07807, msg);
}

void LocalPlanner::printPathDetails(PathOption path, Vector2f goal_loc){
	std::cout << "curvature:        " << path.curvature << std::endl
			  << "clearance:        " << path.clearance << std::endl
			  << "free path length: " << path.free_path_length << std::endl
			  << "distance_to_goal: " << path.distance_to_goal << std::endl
			  << "cost:             " << path.cost << std::endl;
	printVector(path.obstruction, "obstruction: ");
	printVector(path.closest_point, "closest point: ");
	printVector(path.end_point, "end point: ");
	printVector(goal_loc, "goal position");
	std::cout << " - - - - - - - - - - - - - - - " << std::endl;
}

} // namespace navigation