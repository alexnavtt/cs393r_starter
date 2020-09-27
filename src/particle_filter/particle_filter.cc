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

// TODO by Mark
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
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
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
    Vector2f real_reading(p_ptr->loc.x() + ranges.at(i)*cos(laser_angle + p_ptr->angle),
                          p_ptr->loc.y() + ranges.at(i)*sin(laser_angle + p_ptr->angle));
    float diff_sq = (real_reading - predicted_cloud.at(i)).squaredNorm();
    log_error_sum += -diff_sq/var_obs_;

    laser_angle += angle_diff;
  }

  p_ptr->log_weight += log_error_sum;
}

// Done
void ParticleFilter::Resample() {
  // Initialize Local Variables (static for speed in exchange for memory)
  static vector<Particle> new_particles(FLAGS_num_particles);             // temp variable to house new particles
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
    if (absolute_weight_breakpoints[i] > sample_point){
      new_particles.at(i) = particles_[i];
      sample_point += division_size;
    }
  }

  particles_ = new_particles;
}

// TODO by anyone
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
  Resample();
}

// TODO by Connor
void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
}

// TODO by anyone
void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
}

// TODO by anyone
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


}  // namespace particle_filter