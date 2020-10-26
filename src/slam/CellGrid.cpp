#include "CellGrid.h"
#include <iostream>

using Eigen::Vector2f;
using std::array;
using std::vector;
using std::cout;
using std::endl;

// Custom Constructor
CellGrid::CellGrid(Vector2f ORIGIN, float RES, float WIDTH, float HEIGHT) : 
min_cost_(-1000)
{
	origin_ = ORIGIN;
	resolution_ = RES;
	width_ = ceil(WIDTH/RES);
	height_ = ceil(HEIGHT/RES);
	grid_.resize(width_);
	for(auto &row : grid_) row.resize(height_);
}

// Getters
Vector2f CellGrid::getOrigin()     const {return origin_;}
float    CellGrid::getResolution() const {return resolution_;}
int      CellGrid::getXCellCount() const {return width_;}
int      CellGrid::getYCellCount() const {return height_;}
float    CellGrid::getWidth()      const {return width_ * resolution_;}
float    CellGrid::getHeight()     const {return height_ * resolution_;}

// Get the cell index of a location
std::array<int, 2> CellGrid::getIndex(const Vector2f loc) const{
	Vector2f offset = loc - origin_;
	int xi = offset.x() / resolution_;
	int yi = offset.y() / resolution_;
	if ( not checkXLim(xi) or not checkYLim(yi) ) throw std::out_of_range("Outside grid boundaries");
	return array<int,2>({xi, yi});
}

// Get the location of a cell index
Vector2f CellGrid::getLoc(int xi, int yi) const{
	if ( not checkXLim(xi) or not checkYLim(yi) ) throw std::out_of_range("Outside grid boundaries");
	float x = xi * resolution_ + origin_.x();
	float y = yi * resolution_ + origin_.y();
	return Vector2f(x,y);
}

// Retrieve a grid value using a location
float& CellGrid::atLoc(const Vector2f loc){
	array<int,2> indices = getIndex(loc);
	return(grid_[indices[0]][indices[1]]);
}

// Retrieve a grid value using an index
std::vector<float> &CellGrid::operator [](int i){
	return(grid_[i]);
}

// Determine whether an index is in bounds
bool CellGrid::checkXLim(int xi) const {return(xi >= 0 and xi < width_);}
bool CellGrid::checkYLim(int yi) const {return(yi >= 0 and yi < height_);}

// Clear the history in the grid
void CellGrid::clear(){
	for (auto &row : grid_){
	std::fill(row.begin(), row.end(), min_cost_);
	}
}

// Propagate a laser scan's probability distribution
void CellGrid::applyLaserPoint(Vector2f loc, float std_dev){
	array<int,2> startIndex;
	try{
		startIndex = getIndex(loc);
	}catch(std::out_of_range){
		return;
	}
	int x0 = startIndex[0];
	int y0 = startIndex[1];

	float variance = std_dev*std_dev;

	bool too_far_x = false;
	int dxi = 0;
	while (not too_far_x){

		bool too_far_y = false;
		int dyi = 0;
		while (not too_far_y){
			
			try{
				float dx = dxi*resolution_;
				float dy = dyi*resolution_;
				float offset_squared = pow(dx, 2) + pow(dy, 2);
				float log_weight = -offset_squared / variance;

				if ( checkXLim(x0+dxi) and checkYLim(y0+dyi) ) grid_[x0+dxi][y0+dyi] = std::max(log_weight, grid_[x0+dxi][y0+dyi]); // Check the index represented by ( xi,  yi)
				else cout << "skip ++" << endl;
				if ( checkXLim(x0+dxi) and checkYLim(y0-dyi) ) grid_[x0+dxi][y0-dyi] = std::max(log_weight, grid_[x0+dxi][y0-dyi]); // Check the index represented by ( xi, -yi)
				else cout << "skip +-" << endl;
				if ( checkXLim(x0-dxi) and checkYLim(y0+dyi) ) grid_[x0-dxi][y0+dyi] = std::max(log_weight, grid_[x0-dxi][y0+dyi]); // Check the index represented by (-xi,  yi)
				else cout << "skip -+" << endl;
				if ( checkXLim(x0-dxi) and checkYLim(y0-dyi) ) grid_[x0-dxi][y0-dyi] = std::max(log_weight, grid_[x0-dxi][y0-dyi]); // Check the index represented by (-xi, -yi)
				else cout << "skip --" << endl;

				dyi++;
				too_far_y = log_weight < min_cost_;

			}catch(std::out_of_range){
				continue;
			}

		}

	dxi++;
	too_far_x = -pow(dxi*resolution_, 2) / variance < min_cost_;
	}
}

void CellGrid::showGrid(amrl_msgs::VisualizationMsg &viz){
	
	int x_count = 0;
	int y_count = 0;

	for (int xi = 0; xi < width_; xi++){

		if (x_count == 5){
			x_count = 0;
			y_count = 0;

			for (int yi = 0; yi < height_; yi++){
				try{
					Vector2f loc = getLoc(xi, yi);
					float val = grid_[xi][yi];

					if (y_count == 5){
						y_count = 0;

						if (val > 0.8*min_cost_){
							visualization::DrawCross(loc, 0.1*(min_cost_ - val)/min_cost_, 0x000000, viz);
						}
					}

					y_count++;

				}catch(std::out_of_range){
					continue;
				}
			}
		}
		x_count++;
	}
}