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

namespace slam {

SLAM::SLAM() :
		prev_odom_loc_(0, 0),
		prev_odom_angle_(0),
		odom_initialized_(false),
		update_scan_(true),
		// Grid starting at (-10, -10) in the base_link frame with width 20 and height 20 and 0.05m per cell
		prob_grid_({-10,-10}, 0.05, 20, 20),
		prob_grid_init_(false)
{}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
	// Return the latest pose estimate of the robot.
	*loc = Vector2f(0, 0);
	*angle = 0;
}

// Done by Alex
// Returns a point cloud in the base-link frame
vector<Vector2f>* SLAM::Scan2BaseLinkCloud(const LaserScan &s) const{
	// I'm assuming that our laser scan will never change shape here - Alex
	static vector<Vector2f> pointcloud(s.ranges.size());
	static float angle_diff = (s.angle_max - s.angle_min) / s.ranges.size();

	float angle = s.angle_min;
	for (size_t i = 0; i < s.ranges.size(); i++){
		pointcloud[i] = Vector2f(s.ranges[i]*cos(angle) + 0.2, s.ranges[i]*sin(angle));
		angle += angle_diff;
	}

	// FYI: Returning a pointer is only valid for a static variable or a heap allocated variable (i.e. "new" variable) - Alex
	return &pointcloud;
}

// Done by Alex
// Populates the grid with probabilities due to a laser scan
void SLAM::applyScan(LaserScan scan){
	const vector<Vector2f>* points = Scan2BaseLinkCloud(scan);
	prob_grid_.clear();

	int count = 0;
	for (const Vector2f &p : *points){
		if (count != 5){
			count++;
			continue;
		}
		count = 0;
		prob_grid_.applyLaserPoint(p, 0.01);
	}
	prob_grid_init_ = true;
}

// Done by Mark
void SLAM::ObserveLaser(const vector<float>& ranges,
                                     float range_min,
                                     float range_max,
                                     float angle_min,
                                     float angle_max,
                                     amrl_msgs::VisualizationMsg &viz) {
	// A new laser scan has been observed. Decide whether to add it as a pose
	// for SLAM. If decided to add, align it to the scan from the last saved pose,
	// and save both the scan and the optimized pose.

	// NOTE: The reason I check the update_scan_ boolean is to prevent this
	// function from saving the laser scan each time it's called, which would
	// probably slow things down
	if (update_scan_){
		current_scan_ = LaserScan({ranges, range_min, range_max, angle_min, angle_max});
		if (prob_grid_init_)
		{
			// Transform the current scan centered around possible poses from
			// motion model to find best fit with lookup table from previous scan
			ApplyCSM(current_scan_);
		}
		// Get lookup table, preparing for next scan
		applyScan(current_scan_);
		prob_grid_.showGrid(viz);
		update_scan_ = false;
	}
	if (prob_grid_init_) prob_grid_.showGrid(viz);
}

// Done by Alex
Pose SLAM::ApplyCSM(LaserScan scan) {
	float max_cost = -std::numeric_limits<float>::infinity();
	Pose best_pose;

	vector<Vector2f>* base_link_scan = Scan2BaseLinkCloud(scan);

	for (const Pose &pose : possible_poses_){
		float pose_cost = 0.0;

		// Transform laser scan to last pose's base_link frame
		// = ConnorsFunction();

		// Loop through laser points in this scan and add up the log likelihoods
		for (const Vector2f &loc : *base_link_scan){
			try{
				pose_cost += prob_grid_.atLoc(loc);
			}catch(std::out_of_range &e){
				cout << "Scan has no overlap, skipping" << endl;
				continue;
			}
		}

		// Account for motion model probability here (idk how just yet)

		if (pose_cost > max_cost){
			best_pose = pose;
			max_cost = pose_cost;
		}
	}

	return best_pose;
}

// untested
void SLAM::ApplyMotionModel(Eigen::Vector2f loc, float angle, float dist_traveled, float angle_diff) {
	possible_poses_.clear();
	int res = 20;	// number of entries will be res^3, so don't make it too high-res
	
	// Noise constants to tune
	const float k1 = 0.40;  // translation error per unit translation (suggested: 0.1-0.2)
	const float k2 = 0.02;  // translation error per unit rotation    (suggested: 0.01)
	const float k3 = 0.20;  // angular error per unit translation     (suggested: 0.02-0.1)
	const float k4 = 0.40;  // angular error per unit rotation        (suggested: 0.05-0.2)
	// Introduce noise based on motion model
	const float x_stddev = k1*dist_traveled + k2*angle_diff;
	const float y_stddev = k1*dist_traveled + k2*angle_diff;
	const float ang_stddev = k3*dist_traveled + k4*angle_diff;
	// Precalculate trig stuff for rotation
	const float cos_ang = cos(angle);
	const float sin_ang = sin(angle);

	// 3 for loops for each dimension of voxel cube (x, y, theta)
	for (int x_i=0; x_i<res; x_i++)
	{
		float noise_x = x_stddev*(2*x_i/res-1);
		for (int y_i=0; y_i<res; y_i++)
		{
			float noise_y = y_stddev*(2*y_i/res-1);
			for (int ang_i; ang_i<res; ang_i++)
			{
				float pose_ang = angle + ang_stddev*(2*ang_i/res-1);
				float pose_x = loc.x() + noise_x*cos_ang - noise_y*sin_ang;
				float pose_y = loc.y() + noise_x*sin_ang + noise_y*cos_ang;
				Pose this_pose = {{pose_x, pose_y}, pose_ang};
				possible_poses_.push_back(this_pose);
			}
		}
	}
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
		// updates possible_poses_
		ApplyMotionModel(odom_loc, odom_angle, dist_traveled, angle_diff);
		// Save the current laser scan
		update_scan_ = true;
		prev_odom_angle_ = odom_angle;
		prev_odom_loc_ = odom_loc;
	}
}

vector<Vector2f> SLAM::GetMap() {
	vector<Vector2f> map;
	// Reconstruct the map as a single aligned point cloud from all saved poses
	// and their respective scans.
	return map;
}

}  // namespace slam
