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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using math_util::AngleDiff;
using math_util::RadToDeg;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace{
	bool make_new_raster_ = false;
}

namespace slam {

SLAM::SLAM() :
		prev_odom_loc_(0, 0),
		prev_odom_angle_(0),
		odom_initialized_(false),
		prob_grid_({-30,-30}, 0.05, 60, 60)   // Grid starting at (-30, -30) in the base_link frame with width 60 and height 60 and 0.05m per cell
{}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
	// Return the latest pose estimate of the robot.
	*loc = Vector2f(0, 0);
	*angle = 0;
}

// Done by Alex
// Returns a point cloud in the base-link frame
vector<Vector2f> SLAM::Scan2BaseLinkCloud(const LaserScan &s) const{
	vector<Vector2f> pointcloud(s.ranges.size());
	float angle_diff = (s.angle_max - s.angle_min) / s.ranges.size();

	float angle = s.angle_min;
	for (size_t i = 0; i < s.ranges.size(); i++){
		pointcloud[i] = Vector2f(s.ranges[i]*cos(angle) + 0.2, s.ranges[i]*sin(angle));;
		angle += angle_diff;
	}

	return pointcloud;
}

// Done by Alex
// Populates the grid with probabilities due to a laser scan
void SLAM::applyScan(LaserScan scan){
	vector<Vector2f> points = Scan2BaseLinkCloud(scan);
	prob_grid_.clear();

	int count = 0;
	for (const Vector2f &p : points){
		if (count != 5){
			count++;
			continue;
		}
		count = 0;
		prob_grid_.applyLaserPoint(p, 0.03);
	}
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                                     float range_min,
                                     float range_max,
                                     float angle_min,
                                     float angle_max,
                                     amrl_msgs::VisualizationMsg &viz) {
	// A new laser scan has been observed. Decide whether to add it as a pose
	// for SLAM. If decided to add, align it to the scan from the last saved pose,
	// and save both the scan and the optimized pose.

	if (make_new_raster_){
		// Store the scan as our most recent scan
		LaserScan laser_scan = LaserScan({ranges, range_min, range_max, angle_min, angle_max});
		applyScan(laser_scan);
		prob_grid_.showGrid(viz);
		make_new_raster_ = false;
	}

	// This is test code, not permanent - Alex
	// static bool init = false;
	// static double last_time = GetMonotonicTime();
	// double now = GetMonotonicTime();

	// if ((now - last_time) > 0.0){
	// 	applyScan(current_scan_);

	// 	last_time = GetMonotonicTime();
	// 	last_scan_ = current_scan_;
	// 	init = true;
	// }
	// if (init) prob_grid_.showGrid(viz);
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
	if (!odom_initialized_) {
		prev_odom_angle_ = odom_angle;
		prev_odom_loc_ = odom_loc;
		odom_initialized_ = true;
		return;
	}
	// Keep track of odometry to estimate how far the robot has moved between poses.

	float dist_traveled = (odom_loc - prev_odom_loc_).norm();
	float angle_diff = RadToDeg(std::abs(AngleDiff(odom_angle, prev_odom_angle_)));

	if (dist_traveled > 0.5 or angle_diff > 30)
	{
		cout << "New raster!" << endl;
		// Next scan will be rasterized and pose will be updated
		make_new_raster_ = true;
		prev_odom_angle_ = odom_angle;
		prev_odom_loc_ = odom_loc;
	}

	/* 
	--------- Pseudo-Code -----------

	if (moved enough) then
		generate possible locations
		
		for (each location) do
			sum likelihoods from rasterized grid
			record most likely pose
		end

		update grid with most recent scan data
		update all previous poses? (not too sure about this one)
	end

	----------------------------------
	*/
}

vector<Vector2f> SLAM::GetMap() {
	vector<Vector2f> map;
	// Reconstruct the map as a single aligned point cloud from all saved poses
	// and their respective scans.
	return map;
}

}  // namespace slam
