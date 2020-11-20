#include "global_planner.h"

using std::string;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using geometry::line2f;

//========================= GENERAL FUNCTIONS =========================//

void GlobalPlanner::setResolution(float resolution){
	map_resolution_ = resolution;
	cout << "Resolution set to: " << map_resolution_ << endl;
}


//========================= NODE FUNCTIONS ============================//

// Done: Alex
string GlobalPlanner::getNewID(int xi, int yi){
	string id = std::to_string(xi) + "_" +  std::to_string(yi);
	return id;	
}

// Done: Alex
float GlobalPlanner::edgeCost(const Node &node_A, const Node &node_B){
	// Basic distance cost function
	return((node_A.loc - node_B.loc).norm());
}

// Helper Function (untested)
// outputs 2 lines parallel to edge that are displaced by a given offset
std::array<line2f,4> GlobalPlanner::getCushionLines(line2f edge, float offset){
	std::array<line2f, 4> bounding_box;
	Vector2f edge_unit_vector = (edge.p1 - edge.p0)/(edge.p1 - edge.p0).norm();
	Vector2f extended_edge = edge.p1 + offset* edge_unit_vector;   

	Vector2f normal_vec = edge.UnitNormal();
	Vector2f cushion_A_point_1 = edge.p0 + normal_vec * offset;
	Vector2f cushion_A_point_2 = extended_edge + normal_vec * offset;
	Vector2f cushion_B_point_1 = edge.p0 - normal_vec * offset;
	Vector2f cushion_B_point_2 = extended_edge - normal_vec * offset;

	bounding_box[0] = line2f(cushion_A_point_1, cushion_A_point_2);
	bounding_box[1] = line2f(cushion_B_point_1, cushion_B_point_2);
	bounding_box[2] = line2f(cushion_A_point_1, cushion_B_point_1);
	bounding_box[3] = line2f(cushion_A_point_2, cushion_B_point_2);
	return bounding_box;
}

// Done: Alex (untested)
bool GlobalPlanner::isValidNeighbor(const Node &node, const Neighbor &neighbor){
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

	// Check for collisions
	for (const line2f map_line : map_.lines)
    {
		bool intersection = map_line.Intersects(edge);
		for (const line2f bounding_box_edge : cushion_lines){
			intersection = intersection or map_line.Intersects(bounding_box_edge);
		}
		if (intersection) return false;
    }

	return true;
}

// Done: Alex (untested)
vector<Neighbor> GlobalPlanner::getNeighbors(const Node &node){
	vector<Neighbor> neighbors;
	vector<Neighbor> valid_neighbors;

	int xi = node.index.x();
	int yi = node.index.y();

	float diagonal_path_length = sqrt(2)*map_resolution_;	// Assuming a straight line path
	float straight_path_length = map_resolution_;					// Straight line distance

	// Note the each neighbor has the form {ID, curvature, path length, index}
	neighbors.push_back({Vector2i(xi-1, yi+1), getNewID(xi-1, yi+1), diagonal_path_length, 0});	// Left and up
	neighbors.push_back({Vector2i(xi  , yi+1), getNewID(xi,   yi+1), straight_path_length, 1});	// Directly up
	neighbors.push_back({Vector2i(xi+1, yi+1), getNewID(xi+1, yi+1), diagonal_path_length, 2});	// Right and up
	neighbors.push_back({Vector2i(xi-1, yi  ), getNewID(xi-1, yi  ), straight_path_length, 3});	// Directly left
	neighbors.push_back({Vector2i(xi+1, yi  ), getNewID(xi+1, yi  ), straight_path_length, 5});	// Directly right
	neighbors.push_back({Vector2i(xi-1, yi-1), getNewID(xi-1, yi-1), diagonal_path_length, 6}); // Left and down
	neighbors.push_back({Vector2i(xi  , yi-1), getNewID(xi,   yi-1), straight_path_length, 7});	// Directly down
	neighbors.push_back({Vector2i(xi+1, yi-1), getNewID(xi+1, yi-1), diagonal_path_length, 8});	// Right and down

	// Only pass valid neighbors
	for(size_t i=0; i<neighbors.size(); i++){
		if ( isValidNeighbor(node, neighbors[i]) ){
			valid_neighbors.push_back(neighbors[i]);
		}
	}

	return valid_neighbors;
}

// Done: Alex (untested)
Node GlobalPlanner::newNode(const Node &old_node, int neighbor_index){
	Node new_node;
	
	// Change in index, not in position
	int dx = (neighbor_index % 3 == 2) - (neighbor_index % 3 == 0);
	int dy = (neighbor_index < 3) - (neighbor_index > 5);

	new_node.loc 	   = old_node.loc + map_resolution_ * Vector2f(dx, dy);
	new_node.index 	   = old_node.index + Vector2i(dx, dy);
	new_node.cost 	   = old_node.cost + edgeCost(old_node, new_node);
	new_node.parent    = old_node.key;
	new_node.key 	   = getNewID(new_node.index.x(), new_node.index.y());
	new_node.neighbors = getNeighbors(new_node);

	for (const auto &bad_loc : failed_locs_){
		if ((new_node.loc - bad_loc).norm() < map_resolution_*3){
			new_node.neighbors.clear();
			break;
		}
	}

	// Add to node map with key
	nav_map_[new_node.key] = new_node;

	return new_node;
}

// Done: Alex (untested)
void GlobalPlanner::initializeMap(Eigen::Vector2f loc){
	nav_map_.clear();
	frontier_.Clear();

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
	frontier_.Push("START", 0.0);

	// Initialize blueprint map
	map_.Load("maps/GDC1.txt");
	cout << "Initialized GDC1 map with " << map_.lines.size() << " lines." << endl;
}


//========================= PATH PLANNING ============================//

void GlobalPlanner::getGlobalPath(Vector2f nav_goal_loc){
	nav_goal_ = nav_goal_loc;

	bool global_path_success = false;
	int loop_counter = 0; // exit condition if while loop gets stuck (goal unreachable)
	string current_key;
	while(!frontier_.Empty() && loop_counter < 1E6)
	{
		// Get key for the lowest-priority node in frontier_ and then remove it
		current_key = frontier_.Pop();
		Node current_node = nav_map_[current_key];

		// Are we there yet? (0.71 is sqrt(2)/2 with some added buffer)
		if ( (nav_goal_loc - current_node.loc).norm() < 0.71*map_resolution_ )
		{
			global_path_success = true;
			break;
		}

		for(auto &next_neighbor : current_node.neighbors)
		{
			string neighbor_id = next_neighbor.key;
			bool unexplored = !nav_map_.count(next_neighbor.key);
			float neighbor_cost = current_node.cost + next_neighbor.path_length;

			// Is this the first time we've seen this node?
			if (unexplored){
				// Make new Node out of neighbor
				Node new_node = newNode(current_node, next_neighbor.neighbor_index);
				float heuristic = 1.0*getHeuristic(nav_goal_loc, new_node.loc);
				frontier_.Push(neighbor_id, neighbor_cost+heuristic);
			
			}else if (neighbor_cost < nav_map_[neighbor_id].cost){
				nav_map_[neighbor_id].cost = neighbor_cost;
				nav_map_[neighbor_id].parent = current_node.key;
				float heuristic = 1.0*getHeuristic(nav_goal_loc, nav_map_[neighbor_id].loc);
				frontier_.Push(neighbor_id, neighbor_cost+heuristic);

			}
		}
		loop_counter++;
	}

	vector<string> global_path;
	if (global_path_success){
		cout << "After " << loop_counter << " iterations, global path success!" << endl;
		// Backtrace optimal A* path
		string path_key = current_key;
		float total_dist_travelled = 0;
		while (path_key != "START"){
			global_path.push_back(path_key);
			total_dist_travelled += edgeCost(nav_map_[path_key], nav_map_[nav_map_[path_key].parent]);
			path_key = nav_map_[path_key].parent;
		}
		cout << "Travelled " << total_dist_travelled << "m" << endl;
		// If you want to go from start to goal:
		std::reverse(global_path.begin(), global_path.end());
	}
	else{
		cout << "After " << loop_counter << " iterations, global path failure." << endl;
		global_path.push_back("START");
	}

	global_path_ = global_path;
}

float GlobalPlanner::getHeuristic(const Vector2f &goal_loc, const Vector2f &node_loc){
	Vector2f abs_diff_loc = (goal_loc - node_loc).cwiseAbs();
	// 4-grid heuristic is just Manhattan distance
	// float heuristic = abs_diff_loc.x() + abs_diff_loc.y();

	// 2-norm doesn't seem to work either
	// float heuristic = abs_diff_loc.norm();

	// 8-grid heuristic is a little bit complex
	float straight_length = std::abs(abs_diff_loc.x()-abs_diff_loc.y());
	float diag_length = sqrt(2)*(abs_diff_loc.x()+abs_diff_loc.y()-straight_length)*0.5;
	float heuristic = straight_length + diag_length;

	// No hueristic
	// float hueristic = 0;

	return heuristic;
}

// In-work: Connor 
// Post: will actually need to pass in the node location to the drive along global path function
Node GlobalPlanner::getClosestPathNode(Eigen::Vector2f robot_loc, amrl_msgs::VisualizationMsg &msg){
	// Draw Circle around Robot's Location that will Intersect with Global Path
	float circle_rad_min = 2.0;
	visualization::DrawArc(robot_loc,circle_rad_min,0.0,2*M_PI,0x000000, msg);

	// Find the closest node to the robot
	float min_distance = 100;
	Node closest_path_node;
	int starting_point_index = 0;
	float dist_to_node_loc;
	for (size_t i = 0; i < global_path_.size(); i++)
	{
		Vector2f node_loc = nav_map_[global_path_[i]].loc;
		dist_to_node_loc = (robot_loc-node_loc).norm();
		if (dist_to_node_loc < min_distance){
			min_distance = dist_to_node_loc;
			closest_path_node = nav_map_[global_path_[i]];
			starting_point_index = i;
		}
	}
	closest_path_node.visited = true;

	// Check if the closest node is outside circle radius
	need_replan_ = false;
	if (min_distance > circle_rad_min){
		need_replan_ = true;

		// visualization::DrawCross(closest_path_node_outside.loc, 0.25, 0xff9900, msg);
		// visualization::DrawLine(robot_loc, closest_path_node_outside.loc, 0xff9900, msg);
		// std::cout << "min_distance is:\t " << min_distance << std::endl;
		return closest_path_node;
	}

	// Extract the first node after the closest node that is outside the circle
	dist_to_node_loc = min_distance;
	Node closest_path_node_outside;
	for(size_t j = starting_point_index; j < global_path_.size(); j++)
	{
		closest_path_node_outside = nav_map_[global_path_[j]];
		dist_to_node_loc = (robot_loc - closest_path_node_outside.loc).norm();

		// std::cout << "Distance to Next Node in Map is:\t " << dist_to_node_loc << std::endl;
		// visualization::DrawCross(closest_path_node_outside.loc, 0.15, 0xffff00, msg);
		// visualization::DrawLine(robot_loc, closest_path_node_outside.loc, 0xffff00, msg);
		if (dist_to_node_loc > circle_rad_min) break;
	}

	// Draw Closest Point outside circle
	// visualization::DrawCross(closest_path_node_outside.loc, 0.25, 0xff9900, msg);
	// visualization::DrawLine(robot_loc, closest_path_node_outside.loc, 0xff9900, msg);
	return closest_path_node_outside;
}


//========================= VISUALIZATION ============================//

void GlobalPlanner::plotGlobalPath(amrl_msgs::VisualizationMsg &msg){
	if (global_path_.empty()) return;

	Vector2f start = nav_map_[global_path_.front()].loc;
	Vector2f goal = nav_map_[global_path_.back()].loc;
	visualization::DrawCross(start, 0.5, 0xff0000, msg);
	visualization::DrawCross(goal, 0.5, 0xff0000, msg);

	for (auto key = global_path_.begin(); key != global_path_.end(); key++){
		string end_key = nav_map_[*key].parent;
		Vector2f start_loc = nav_map_[*key].loc;
		Vector2f end_loc = nav_map_[end_key].loc;
		visualization::DrawLine(start_loc, end_loc, 0x009c08, msg);
	}
}

void GlobalPlanner::plotFrontier(amrl_msgs::VisualizationMsg &msg){
	while(!frontier_.Empty()){
		string frontier_key = frontier_.Pop();
		Vector2f frontier_loc = nav_map_[frontier_key].loc;
		visualization::DrawPoint(frontier_loc, 0x0000ff, msg);
	}
}

// Done: Connor
void GlobalPlanner::plotNodeNeighbors(const Node &node, amrl_msgs::VisualizationMsg &msg){
	// Visualize the node and it's immediate neighbors

	visualization::DrawCross(node.loc,2.0,0xff0000,msg);
	for (size_t i = 0; i < node.neighbors.size(); i++){
		// Get the ID for this neighboring node
		string neighbor_id = node.neighbors[i].key;
		int neighbor_index = node.neighbors[i].neighbor_index;

		// Find the location of the neighbor
		int dx = (neighbor_index % 3 == 2) - (neighbor_index % 3 == 0);
		int dy = (neighbor_index < 3) - (neighbor_index > 5);
		Vector2f neighbor_loc = node.loc + map_resolution_ * Vector2f(dx, dy);

		// Visualize
		visualization::DrawPoint(neighbor_loc,0xff9900,msg);
		visualization::DrawLine(node.loc, neighbor_loc, 0x000dff, msg);		
	}
}

bool GlobalPlanner::needsReplan(){return need_replan_;}

void GlobalPlanner::replan(Vector2f robot_loc, Vector2f failed_target_loc){
	// Find the last visited node
	// for (const string &id : global_path_){
	// 	if (not nav_map_[id].visited){
	// 		nav_map_[nav_map_[id].parent].key = "START";
	// 		nav_map_["START"] = nav_map_[nav_map_[id].parent];
	// 		break;
	// 	}
	// }
	failed_locs_.push_back(failed_target_loc);
	initializeMap(robot_loc);

	// Unvisit all nodes
	// for (auto it = nav_map_.begin(); it != nav_map_.end(); it++){
	// 	it->second.visited = false;
	// }
	// nav_map_["START"].visited = true;

	// Clear the global path and start fresh from the current node
	// global_path_.clear();
	// frontier_.Clear();
	// frontier_.Push("START", 0.0);
	// global_path_.push_back("START");

	// Invalidate the the node we were trying to get to when navigation failed (and it's neighbors)
	// for (Neighbor &n : nav_map_[failed_target_id].neighbors){
	// 	if (nav_map_.count(n.key))
	// 		nav_map_[n.key].neighbors.clear();
	// }
	// nav_map_[failed_target_id].neighbors.clear();

	getGlobalPath(nav_goal_);

	cout << "replanning and avoiding nodes at:" << endl;
	for (auto &l : failed_locs_){
		cout << "(" << l.x() << ", " << l.y() << ")" << endl;
	}
	cout << endl;
}
