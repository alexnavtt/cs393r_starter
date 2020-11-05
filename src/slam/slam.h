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

// Custom Class
#include "CellGrid.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Pose{
  Eigen::Vector2f loc;
  float angle;
};

struct PoseWithLikelihood{
  Pose pose;
  float log_likelihood;
};

struct LaserScan{
  std::vector<float> ranges;
  float range_min;
  float range_max;
  float angle_min;
  float angle_max;
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
                    float angle_max,
                    amrl_msgs::VisualizationMsg &viz);

  // does what the name says it does
  Eigen::Vector2f TransformNewScanToPrevPose(const Eigen::Vector2f scan_loc, Pose odom_pose_cur);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Update map
  void updateMap(Pose pose);
  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  // Convert a laser scan to a point cloud
  std::vector<Eigen::Vector2f>* Scan2MapCloud(const LaserScan &s) const;
  std::vector<Eigen::Vector2f>* Scan2BaseLinkCloud(const LaserScan &s) const;

  // Get distribution of possible robot poses
  void ApplyMotionModel(Eigen::Vector2f loc, float angle, float dist_traveled, float angle_diff);

  // Store a scan as a prior in the probability grid
  void applyScan(LaserScan s);

  // Apply Correlative Scan Matching Algorithm
  Pose ApplyCSM(LaserScan s);

 private:

  // Tuning parameters
  float observation_likelihood_res_;
  float observation_likelihood_std_dev_;
  float motion_model_weight_;
  float laser_scan_weight_;
  int CSM_scan_offset_;
  float x_res_;
  float y_res_;
  float t_res_;
  float k1_;
  float k2_;
  float k3_;
  float k4_;
  float linear_diff_threshold_;
  float angular_diff_threshold_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;
  Eigen::Rotation2Df R_odom2MLE_;
  Pose MLE_pose_;
  Pose current_pose_;

  // Storing scans
  LaserScan current_scan_;   // Current scan for SLAM algorithm to use
  bool update_scan_;         // Flag for whether or not to update the SLAM map

  // Motion model variables
  std::vector<PoseWithLikelihood> possible_poses_;

  // Rasterized grid
  CellGrid prob_grid_;
  bool prob_grid_init_;   // Flag to fix vizualization bug where the entire grid is shown on startup

  // Map vector
  std::vector<Eigen::Vector2f> map_scans_;
};

}  // namespace slam

#endif   // SRC_SLAM_H_
