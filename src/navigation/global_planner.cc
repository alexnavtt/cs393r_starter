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
std::array<line2f,2> GlobalPlanner::getCushionLines(line2f edge, float offset){
	Vector2f normal_vec = edge.UnitNormal();
	Vector2f cushion_A_point_1 = edge.p0 + normal_vec * offset;
	Vector2f cushion_A_point_2 = edge.p1 + normal_vec * offset;
	Vector2f cushion_B_point_1 = edge.p0 - normal_vec * offset;
	Vector2f cushion_B_point_2 = edge.p1 - normal_vec * offset;
	return {line2f(cushion_A_point_1, cushion_A_point_2), line2f(cushion_B_point_1, cushion_B_point_2)};
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
		bool intersects_main = map_line.Intersects(edge);
		bool intersects_cushion_0 = map_line.Intersects(cushion_lines[0]);
		bool intersects_cushion_1 = map_line.Intersects(cushion_lines[1]);
		if (intersects_main or intersects_cushion_0 or intersects_cushion_1)
			return false;
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
	new_node.index 	   = old_node.index + Eigen::Vector2i(dx, dy);
	new_node.cost 	   = old_node.cost + edgeCost(old_node, new_node);
	new_node.parent    = old_node.key;
	new_node.key 	   = getNewID(new_node.index.x(), new_node.index.y());
	new_node.neighbors = getNeighbors(new_node);

	// Add to node map with key
	nav_map_[new_node.key] = new_node;

	return new_node;
}

// Done: Alex (untested)
void GlobalPlanner::initializeMap(Eigen::Vector2f loc){
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
	frontier_.Push("START", 0.0);
}


//========================= PATH PLANNING ============================//

vector<string> GlobalPlanner::getGlobalPath(Vector2f nav_goal_loc){
	bool global_path_success = false;
	int loop_counter = 0; // exit condition if while loop gets stuck (goal unreachable)
	while(!frontier_.Empty() && loop_counter < 1000)
	{
		// Get key for the lowest-priority node in frontier_ and then remove it
		string current_key = frontier_.Pop();
		Node current_node = nav_map_[current_key];

		// Are we there yet?
		if ( (nav_goal_loc - current_node.loc).norm() < map_resolution_/2 )
		{
			global_path_success = true;
			break;
		}

		for(auto &next_neighbor : current_node.neighbors)
		{
			float new_cost = current_node.cost + next_neighbor.path_length;
			// Is this the first time we've seen this node?
			bool unexplored = !nav_map_.count(next_neighbor.key);
			if (unexplored){
				// Make new Node out of neighbor
				Node new_node = newNode(current_node, next_neighbor.neighbor_index);
			}
			// Assume that neighbors and nodes at the same location have the same keys
			if (unexplored or (new_cost < nav_map_[next_neighbor.key].cost))
			{
				nav_map_[next_neighbor.key].cost = new_cost;
				// L1 norm or Manhattan distance (change to 1.05 or 1.1 if you want to inflate it)
				float heuristic = 1.0*(nav_goal_loc - nav_map_[next_neighbor.key].loc).lpNorm<1>();
				frontier_.Push(next_neighbor.key, new_cost+heuristic);
			}
		}
		loop_counter++;
	}
	vector<string> global_path;
	if (global_path_success){
		cout << "Global path success!" << endl;
		// Backtrace optimal A* path
		string path_key = current_node.key;
		while (path_key != "START"){
			global_path.push_back(path_key);
			path_key = nav_map_[path_key].parent;
		}
		// If you want to go from start to goal:
		std::reverse(global_path.begin(), global_path.end());
	}
	else{
		cout << "Global path failure." << endl;
		global_path.push_back("START");
	}
	return global_path;
}


//========================= VISUALIZATION ============================//

// Done: Connor
void GlobalPlanner::plotNodeNeighbors(const Node &node, amrl_msgs::VisualizationMsg &msg){
	// Visualize the node and it's immediate neighbors using msg (defined in line 52)
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
	visualization::DrawCross(node.loc,2.0,0xff0000,msg);
	for (size_t i = 0; i < node.neighbors.size(); i++){
		// Get the ID for this neighboring node
		string neighbor_id = node.neighbors[i].key;
		// // Check if node already exists
		// if (not nav_map_.count(neighbor_id)){
		// 	// If it does not, create a temporary new one
		// 	Node neighbor = newNode(node, i);
		// }
		int neighbor_index = node.neighbors[i].neighbor_index;
		int dx = (neighbor_index % 3 == 2) - (neighbor_index % 3 == 0);
		int dy = (neighbor_index < 3) - (neighbor_index > 5);
		Vector2f neighbor_loc = node.loc + map_resolution_ * Vector2f(dx, dy);
		visualization::DrawPoint(neighbor_loc,0xff9900,msg);
		visualization::DrawLine(node.loc, neighbor_loc, 0x000dff, msg);
		// Do the plot stuff here
		
	}
}
