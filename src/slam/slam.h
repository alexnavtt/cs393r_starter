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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Pose{
  Eigen::Vector2f loc;
  float angle;
};

struct LaserScan{
  std::vector<float> ranges;
  float range_min;
  float range_max;
  float angle_min;
  float angle_max;
};

class CellGrid{
public:
  Eigen::Vector2f origin;   // Location of the center of the lower left block
  float resolution;         // Size of grid blocks in meters (grid blocks are square)
  int width;                // Width of the grid in cells
  int height;               // Height of the grid in cells
  Pose reference_pose;      // The original pose that resulted in the current grid

  std::vector< std::vector<float> > grid;   // Grid of log-likelihoods

  // Retrieve a grid value using a location
  float &at(const Eigen::Vector2f v){
    Eigen::Vector2f offset = v - origin;
    int dx = offset.x() / resolution;
    int dy = offset.y() / resolution;
    return(grid[dx][dy]);
  }

  // Retrieve a grid value using an index
  std::vector<float> &operator [](int i){
    return(grid[i]);
  }

  // Constructors
  CellGrid(){}
  CellGrid(Eigen::Vector2f ORIGIN, float RES, float WIDTH, float HEIGHT){
    origin = ORIGIN;
    resolution = RES;
    width = ceil(WIDTH/RES);
    height = ceil(HEIGHT/RES);
    grid.resize(width);
    for(auto &row : grid) row.resize(height);
  }
};




class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  // Convert a laser scan to a point cloud
  std::vector<Eigen::Vector2f> Scan2MapCloud(LaserScan s) const;
  std::vector<Eigen::Vector2f> Scan2BaseLinkCloud(LaserScan s) const;

  // Get distribution of possible robot poses
  std::vector<Pose> ApplyMotionModel(Eigen::Vector2f loc, float angle);

  // Apply Correlative Scan Matching Algorithm
  void ApplyCSM();

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // Storing scans
  std::vector<LaserScan> stored_scans_;

  // Rasterized grid
  CellGrid prob_grid_;
};

}  // namespace slam

#endif   // SRC_SLAM_H_
