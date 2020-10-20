#include "CellGrid.h"

using Eigen::Vector2f;
using std::array;
using std::vector;

// Custom Constructor
CellGrid::CellGrid(Vector2f ORIGIN, float RES, float WIDTH, float HEIGHT){
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
  int dx = offset.x() / resolution_;
  int dy = offset.y() / resolution_;
  return array<int,2>({dx, dy});
}

// Get the location of a cell index
Vector2f CellGrid::getLoc(int xi, int yi) const{
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
bool CellGrid::checkXLim(int xi) {return(xi >= 0 and xi < width_);}
bool CellGrid::checkYLim(int yi) {return(yi >= 0 and yi < height_);}

// Propagate a laser scan's probability distribution
void CellGrid::applyLaserPoint(Vector2f loc, float std_dev, float cutoff){
  array<int,2> startIndex = getIndex(loc);
  int x0 = startIndex[0];
  int y0 = startIndex[1];

  float variance = std_dev*std_dev;

  bool too_far_x = false;
  int xi = x0;
  while (not too_far_x){

    bool too_far_y = false;
    int yi = y0;
    while (not too_far_y){
      
      float dx = (x0 - xi)*resolution_;
      float dy = (y0 - yi)*resolution_;
      float offset_squared = pow(dx, 2) + pow(dy, 2);
      float log_weight = -offset_squared / variance;

      if ( checkXLim( xi) and checkYLim( yi) ) grid_[ xi][ yi] = std::max(log_weight, grid_[ xi][ yi]); // Check the index represented by ( xi,  yi)
      if ( checkXLim( xi) and checkYLim(-yi) ) grid_[ xi][-yi] = std::max(log_weight, grid_[ xi][-yi]); // Check the index represented by ( xi, -yi)
      if ( checkXLim(-xi) and checkYLim( yi) ) grid_[-xi][ yi] = std::max(log_weight, grid_[-xi][ yi]); // Check the index represented by (-xi,  yi)
      if ( checkXLim(-xi) and checkYLim(-yi) ) grid_[-xi][-yi] = std::max(log_weight, grid_[-xi][-yi]); // Check the index represented by (-xi, -yi)

      yi++;
      too_far_y = log_weight < cutoff;

    }

    xi++;
    too_far_x = -pow((x0 - xi)*resolution_, 2) / variance < cutoff;
  }

}