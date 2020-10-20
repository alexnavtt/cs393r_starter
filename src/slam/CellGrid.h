#ifndef CELL_GRID_CS393_HH
#define CELL_GRID_CS393_HH

#include <algorithm>
#include <vector>
#include <array>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

class CellGrid{
private:
  Eigen::Vector2f origin_;   // Location of the center of the lower left block
  float resolution_;         // Size of grid blocks in meters (grid blocks are square)
  int width_;                // Width of the grid in cells
  int height_;               // Height of the grid in cells

  std::vector< std::vector<float> > grid_;   // Grid of log-likelihoods

public:
  // Default Constructor
  CellGrid(){}
  // Custom Constructor
  CellGrid(Eigen::Vector2f ORIGIN, float RES, float WIDTH, float HEIGHT);

  // Getters
  Eigen::Vector2f getOrigin() const;
  float getResolution() const;
  int getXCellCount() const;
  int getYCellCount() const;
  float getWidth() const;
  float getHeight() const;

  std::array<int,2> getIndex(Eigen::Vector2f loc) const;
  Eigen::Vector2f getLoc(int x, int y) const;

  // Retrieve a grid value using a location
  float &atLoc(const Eigen::Vector2f loc);

  // Retrieve a grid value using an index
  std::vector<float> &operator [](int i);

  // Check if a cell is within grid boundaries
  bool checkXLim(int xi);
  bool checkYLim(int yi);

  void applyLaserPoint(Eigen::Vector2f loc, float std_dev, float cutoff); 
};

#endif
