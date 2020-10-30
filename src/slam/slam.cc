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
using Eigen::Vector3f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

// ====================  Tunable Parameters =================
//
// -> prob_grid_.applyLaserPoint std_dev (currently 0.025m)
// -> how many scans to use when measuring cost for CSM
// -> motion model resolution (x, y , theta)
// -> motion model k values
// -> resample frequency (angular and linear)
//
// ==========================================================

void trimScan(vector<Vector2f>* cloud, int offset_count){
	for (size_t i = 0; i < cloud->size()/offset_count; i++){
		(*cloud)[i] = (*cloud)[i*offset_count];
	}
	cloud->resize(cloud->size()/offset_count);
}

namespace slam {

SLAM::SLAM() :
		prev_odom_loc_(0, 0),
		prev_odom_angle_(0),
		odom_initialized_(false),
		update_scan_(false),
		// Grid starting at (-8, -8) in the base_link frame with width 16m and height 16m and 0.01m per cell
		prob_grid_({-8,-8}, 0.01, 16, 16),
		prob_grid_init_(false)
{
	map_scans_.reserve(1000000);
}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
	// Return the latest pose estimate of the robot.
	*loc = current_pose_.loc;
	*angle = current_pose_.angle;
}

// Done by Alex
// Returns a point cloud in the base-link frame
vector<Vector2f>* SLAM::Scan2BaseLinkCloud(const LaserScan &s) const{
	// I'm assuming that our laser scan will never change shape here - Alex
	static vector<Vector2f> pointcloud(s.ranges.size());
	static float angle_diff = (s.angle_max - s.angle_min) / s.ranges.size();
	pointcloud.resize(s.ranges.size());

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

	for (const Vector2f &p : *points){
		prob_grid_.applyLaserPoint(p, 0.025);
	}
	prob_grid_init_ = true;
}

// Done by Alex
Pose SLAM::ApplyCSM(LaserScan scan) {
	float max_cost = -std::numeric_limits<float>::infinity();
	Pose best_pose = {{0,0},0};

	vector<Vector2f>* base_link_scan = Scan2BaseLinkCloud(scan);
	trimScan(base_link_scan, 10);

	for (const Pose &pose : possible_poses_){
		float pose_cost = 0.0;
		
		// Loop through laser points in this scan and add up the log likelihoods
		for (const Vector2f &loc : *base_link_scan){

			try{
				// Transform laser scan to last pose's base_link frame
				Vector2f mapped_loc = TransformNewScanToPrevPose(loc, pose);
				pose_cost += prob_grid_.atLoc(mapped_loc);

			}catch(std::out_of_range &e){
				continue;
			}
		}

		if (pose_cost > max_cost){
			best_pose = pose;
			max_cost = pose_cost;
		}
	}

	return best_pose;
}

// Done by Connor
Eigen::Vector2f SLAM::TransformNewScanToPrevPose(const Eigen::Vector2f scan_loc, Pose pose_cur)

{
    // 1st Transform, hard-coded but that's ok
    // Affine2f H_Lidar2BaseCur = GetTransform({{0.2,0.0},0});
    // Affine2f H_Lidar2BaseCur = Eigen::Affine2f::Identity();

    // 2nd Transformm takes in parameters from function call
	// Uncomment if want to test with locally defined instances of previous loc and angle
	// will need to remove _ from their use in trans_diff and angle_diff equations
    // Vector2f prev_odom_loc = {0,0};
    // float prev_odom_angle = M_PI / 4;
    Vector2f odom_trans_diff = pose_cur.loc - MLE_pose_.loc;			 // In the map frame
    float odom_angle_diff = AngleDiff(pose_cur.angle, MLE_pose_.angle);	 // Frame independent
    // Pose diff_pose = {odom_trans_diff, odom_angle_diff};
    // Affine2f H_BaseCur2BasePrev = GetTransform(diff_pose);

    // // Apply transformations to Scan Point
    // Vector3f Scan2Lidar_xyz(scan_loc.x(),scan_loc.y(),1) ;
    // // Vector3f Scan2BasePrev_xyz = H_BaseCur2BasePrev*H_Lidar2BaseCur*Scan2Lidar_xyz;
    // Vector3f Scan2BasePrev_xyz = H_BaseCur2BasePrev*Scan2Lidar_xyz;
    // Vector2f Scan2BasePrev(Scan2BasePrev_xyz.x(),Scan2BasePrev_xyz.y());
    // return Scan2BasePrev;

    Eigen::Rotation2Df R_map2oldBaseLink(-MLE_pose_.angle);
    // Eigen::Rotation2Df R_newBaseLink2map(pose_cur.angle);
    Eigen::Rotation2Df R_newBaseLink2oldBaseLink(odom_angle_diff);
    // Vector2f mapped_scan = R_map2oldBaseLink * odom_trans_diff + R_newBaseLink2map * R_map2oldBaseLink * scan_loc;
    Vector2f mapped_scan = R_map2oldBaseLink * odom_trans_diff + R_newBaseLink2oldBaseLink * scan_loc;
    return mapped_scan;
}

// Eigen::Affine2f SLAM::GetTransform(Pose pose) {
//     // in radians
//     Rotation2Df R(pose.angle);
//     Vector2f Tr = pose.loc;
//     Affine2f H = Eigen::Affine2f::Identity();
//     H.rotate ( R ) ;
//     H.translate ( Tr ) ;
//     // cout << "H = " << endl << H.matrix() << endl;
// 	return H;
// }

// Done by Mark
void SLAM::ObserveLaser(const vector<float>& ranges,
                                     float range_min,
                                     float range_max,
                                     float angle_min,
                                     float angle_max,
                                     amrl_msgs::VisualizationMsg &viz) {
	// Test whether we need to update the map with the current laser scan
	bool apply_scan_flag = update_scan_ or (odom_initialized_ and not prob_grid_init_);

	if (apply_scan_flag){
		current_scan_ = LaserScan({ranges, range_min, range_max, angle_min, angle_max});
		
		// Transform the current scan centered around possible poses from
		// motion model to find best fit with lookup table from previous scan
		MLE_pose_ = ApplyCSM(current_scan_);
		updateMap(MLE_pose_);
		R_odom2MLE_ = Eigen::Rotation2Df(MLE_pose_.angle - prev_odom_angle_);
		
		// Get lookup table, preparing for next scan
		applyScan(current_scan_);
		update_scan_ = false;
	}
	// if (prob_grid_init_) prob_grid_.showGrid(viz);
}

// Done by Mark untested
void SLAM::ApplyMotionModel(Eigen::Vector2f loc, float angle, float dist_traveled, float angle_diff) {
	possible_poses_.clear();
	// int res = 9.0;	// number of entries will be res^3, so don't make it too high-res
	float x_res = 11.0;
	float y_res = 7.0;
	float t_res = 21.0;

	// Noise constants to tune
	const float k1 = 0.40;// 0.40;  // translation error per unit translation (suggested: 0.1-0.2)
	const float k2 = 0.05;// 0.02;  // translation error per unit rotation    (suggested: 0.01)
	const float k3 = 0.10;// 0.20;  // angular error per unit translation     (suggested: 0.02-0.1)
	const float k4 = 0.40;// 0.40;  // angular error per unit rotation        (suggested: 0.05-0.2)
	// Introduce noise based on motion model
	const float abs_angle_diff = abs(angle_diff);
	const float x_stddev = k1*dist_traveled + k2*abs_angle_diff;
	const float y_stddev = k1*dist_traveled + k2*abs_angle_diff;
	const float ang_stddev = k3*dist_traveled + k4*abs_angle_diff;
	// Precalculate trig stuff for rotation
	const float cos_ang = cos(angle);
	const float sin_ang = sin(angle);

	// 3 for loops for each dimension of voxel cube (x, y, theta)
	for (int x_i=0; x_i<x_res; x_i++)
	{
		float noise_x = 2*x_stddev*(2*x_i/(x_res-1) - 1);
		for (int y_i=0; y_i<y_res; y_i++)
		{
			float noise_y = 2*y_stddev*(2*y_i/(y_res-1) - 1);
			for (int ang_i=0; ang_i<t_res; ang_i++)
			{
				float pose_ang = angle + 2*ang_stddev*(2*ang_i/(t_res-1) - 1);
				float pose_x = loc.x() + noise_x*cos_ang - noise_y*sin_ang;
				float pose_y = loc.y() + noise_x*sin_ang + noise_y*cos_ang;
				Pose this_pose = {{pose_x, pose_y}, pose_ang};
				possible_poses_.push_back(this_pose);
			}
		}
	}
}

// Done by Mark
void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
	if (!odom_initialized_) {
		prev_odom_angle_ = odom_angle;
		prev_odom_loc_ = odom_loc;
		odom_initialized_ = true;
		update_scan_ = true;
		return;
	}

	// Keep track of odometry to estimate how far the robot has moved between poses.
	Vector2f odom_diff = odom_loc - prev_odom_loc_;
	float angle_diff = AngleDiff(odom_angle, prev_odom_angle_);
	float dist_traveled = odom_diff.norm();

	// Update current estimate of pose
	current_pose_.loc = MLE_pose_.loc + R_odom2MLE_ * odom_diff;
	current_pose_.angle = fmod(MLE_pose_.angle + angle_diff + M_PI, 2*M_PI) - M_PI;

	if (dist_traveled > 0.15 or abs(angle_diff) > M_PI/24)
	{
		// updates possible_poses_
		ApplyMotionModel(current_pose_.loc, current_pose_.angle, dist_traveled, angle_diff);
		// Save the current laser scan
		update_scan_ = true;
		prev_odom_angle_ = odom_angle;
		prev_odom_loc_ = odom_loc;
	}
}

// Done by Mark
void SLAM::updateMap(Pose CSM_pose) {
	// Reconstruct the map as a single aligned point cloud from all saved poses
	// and their respective scans.
	const int num_ranges = current_scan_.ranges.size();
	const float angle_increment = (current_scan_.angle_max - current_scan_.angle_min) / num_ranges;

	// Transform scan ranges to pose frame
	for(int i = 0; i<num_ranges; i++)
	{
		const float range_i = current_scan_.ranges[i];
		const float angle_i = CSM_pose.angle + current_scan_.angle_min + angle_increment * i;
		const float point_i_x = CSM_pose.loc.x() + range_i*cos(angle_i);
		const float point_i_y = CSM_pose.loc.y() + range_i*sin(angle_i);
		const Vector2f point_i(point_i_x, point_i_y);
		map_scans_.push_back(point_i);
	}
}

vector<Vector2f> SLAM::GetMap() {
	int jump_size = ceil(map_scans_.size() / 5000);
	int N = std::min(int(map_scans_.size()), 5000);
	vector<Vector2f> output_vector(N);

	for (int i = 0; i < N; i++){
		output_vector[i] = map_scans_[i * jump_size];
	}
	return output_vector;
}

}  // namespace slam
