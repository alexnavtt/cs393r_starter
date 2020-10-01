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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using math_util::DegToRad;
using math_util::RadToDeg;
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

// TODO by Connor
void ParticleFilter::UpdateParticleLocation(float dx_odom, float dy_odom, float dtheta_odom, Particle& particle)
{
  // Use the motion model to update each particle's location
  // This function will probably be called in the ObserveOdometry callback
  // You can update the particle location directly by modifying the particle variable
  // defined above since it was passed by reference (using the "&" symbol).

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  float x = rng_.Gaussian(0.0, 2.0);
  printf("Random number drawn from Gaussian distribution with 0 mean and "
         "standard deviation of 2 : %f\n", x);
}

// Done by Mark, untested
void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  
  // Sweeps through angles of virtual Lidar and returns closest point
  for (size_t i_scan = 0; i_scan < scan.size(); ++i_scan)
  {
    // Initialize scan, to be updated later
    scan[i_scan] = Vector2f(0, 0);
    // Get the visual "ray" vector for this particular scan
    line2f ray_line(1,2,3,4); // Line segment from (1,2) to (3,4)
    float ray_angle = angle + i_scan/num_ranges*(angle_max-angle_min) - angle_min;
    ray_line.p0.x() = loc.x() + range_min*cos(ray_angle);
    ray_line.p0.y() = loc.y() + range_min*sin(ray_angle);
    ray_line.p1.x() = loc.x() + range_max*cos(ray_angle);
    ray_line.p1.y() = loc.y() + range_max*sin(ray_angle);
    // printf("P0: %f, %f P1: %f,%f\n", 
    //        ray_line.p0.x(),
    //        ray_line.p0.y(),
    //        ray_line.p1.x(),
    //        ray_line.p1.y());
    
    // Initialize variables for next loop
    Vector2f intersection_min;
    intersection_min.x() = loc.x() + range_max*cos(ray_angle);
    intersection_min.y() = loc.y() + range_max*sin(ray_angle);
    float dist_to_intersection_min = range_max;
    // Sweeps through lines in map to get closest intersection with ray
    for (size_t i_line = 0; i_line < map_.lines.size(); ++i_line)
    {
      const line2f map_line = map_.lines[i_line];
      Vector2f intersection_point;
      bool intersects = map_line.Intersection(ray_line, &intersection_point);
      // If there is an intersection, examine this point
      if (intersects)
      {
        float dist_to_this_intersection = (intersection_point-loc).norm();
        // If it's the closest point yet, record it
        if (dist_to_this_intersection < dist_to_intersection_min)
        {
          dist_to_intersection_min = dist_to_this_intersection;
          intersection_min = intersection_point;
        }
      }
    }
    // Return closest point for this particular scan
    // NOTE: I think this should be put in the base_link frame since that is
    //       how the world is observed with the physical Lidar. However, I'm
    //       leaving it in the map frame at least for now to visualize easier.
    scan[i_scan] = intersection_min;

    // Optional: If you just want the range
    // scan[i_scan] = dist_to_intersection_min;

    // Optional: In base_link frame (untested):
    // scan[i_scan] = Map2BaseLink(intersection_min, loc, angle);
  }
}

// TODO by Alex: Implement d_min and d_max piecewise function
void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Get predicted point cloud
  vector<Vector2f> predicted_cloud;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle,
                         ranges.size(),
                         range_min, range_max,
                         angle_min, angle_max,
                         &predicted_cloud);

  // Calculate Particle Weight (pure Gaussian to start off)
  float log_error_sum = 0;
  float laser_angle = angle_min;
  float angle_diff = (angle_max - angle_min)/ranges.size();

  for (size_t i = 0; i < ranges.size(); i++)
  {
    Vector2f real_reading(p_ptr->loc.x() + ranges[i]*cos(laser_angle + p_ptr->angle),
                          p_ptr->loc.y() + ranges[i]*sin(laser_angle + p_ptr->angle));
    float diff_sq = (real_reading - predicted_cloud[i]).squaredNorm();
    log_error_sum += -diff_sq/var_obs_;

    laser_angle += angle_diff;
  }

  p_ptr->log_weight += log_error_sum;
}

// Done by Alex
void ParticleFilter::Resample() {
  // Initialize Local Variables (static for speed in exchange for memory)
  vector<Particle> new_particles;                                         // temp variable to house new particles
  static vector<float> normalized_log_weights(FLAGS_num_particles);       // vector of log(w/w_max) = log(w) - log(w_max)
  static vector<float> absolute_weight_breakpoints(FLAGS_num_particles);  // vector of cumulative absolute normalized weights
  float normalized_sum = 0;                                               // sum of normalized (but NOT log) weights: used for resampling

  // Normalize each of the log weights
  for (size_t i=0; i < FLAGS_num_particles; i++){
    normalized_log_weights[i] = particles_[i].log_weight - max_log_particle_weight_;
    normalized_sum += exp(normalized_log_weights[i]);
    absolute_weight_breakpoints[i] = normalized_sum;
  }

  // Resample based on the absolute weights
  float division_size = normalized_sum / FLAGS_num_particles;
  float sample_point = rng_.UniformRandom(0,division_size);

  for (size_t i=0; i < FLAGS_num_particles; i++){
    while (absolute_weight_breakpoints[i] > sample_point){
      new_particles.push_back(particles_[i]);
      sample_point += division_size;
    }
  }

  particles_ = new_particles;
}

// Done by Alex
// Called by LaserCallback in particle_filter_main
void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)

  max_log_particle_weight_ = -std::numeric_limits<float>::infinity(); // Since the range of weights is (-inf,0] we have to initialize max at -inf

  // Update all particle weights and find the maximum weight
  for (auto &particle : particles_){
    Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
    if (particle.log_weight > max_log_particle_weight_) max_log_particle_weight_ = particle.log_weight;
  }

  // Resample (we will probably want to stagger this for efficiency)
  // Resample();
}

// TODO by Connor
void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
}

// Done by Mark, untested
// Called by InitCallback in particle_filter_main
void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  particles_.clear(); // Need to get rid of particles from previous inits
  map_.Load("maps/" + map_file + ".txt"); // from Piazza

  // Make initial guesses (particles) based on a Gaussian distribution about initial placement
  for (size_t i = 0; i < FLAGS_num_particles; i++){
    Particle particle_init;
    particle_init.loc.x() = rng_.Gaussian(loc.x(), 0.25);  // std_dev of 0.25m, to be tuned
    particle_init.loc.y() = rng_.Gaussian(loc.y(), 0.25);  // std_dev of 0.25m, to be tuned
    particle_init.angle = rng_.Gaussian(angle, M_PI/6);    // std_dev of 30deg, to be tuned
    particles_.push_back(particle_init);
  }
}

// TODO by anyone
// Called by OdometryCallback in particle_filter_main
void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}

// Helper function to convert from map to base_link, untested
Vector2f ParticleFilter::Map2BaseLink(const Vector2f& point, const Vector2f& loc, const float angle){
  Eigen::Rotation2Df R_inv(-angle); // negative of angle should be the same as transpose or inverse, right?
  Vector2f lidar_reading = R_inv*(point-loc); // transformation to lidar frame
  Vector2f lidar_offset(0.2, 0);
  return lidar_reading - lidar_offset; // transformation to base_link frame
}

}  // namespace particle_filter
