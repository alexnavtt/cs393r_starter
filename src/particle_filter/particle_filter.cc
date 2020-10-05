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
    odom_initialized_(false),
    var_obs_(1),
    d_short_(0.5),
    d_long_(0.2),
    d_min_(-2),
    d_max_(2) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
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

  Vector2f lidar_loc = loc;
  lidar_loc.x() += 0.2*cos(angle);
  lidar_loc.y() += 0.2*sin(angle);
  
  // Sweeps through angles of virtual Lidar and returns closest point
  for (size_t i_scan = 0; i_scan < scan.size(); ++i_scan)
  {
    // Initialize scan, to be updated later
    scan[i_scan] = Vector2f(0, 0);
    // Get the visual "ray" vector for this particular scan
    line2f ray_line(1,2,3,4); // Line segment from (1,2) to (3,4)
    float ray_angle = angle + 1.0*i_scan/num_ranges*(angle_max-angle_min) + angle_min;
    ray_line.p0.x() = lidar_loc.x() + range_min*cos(ray_angle);
    ray_line.p0.y() = lidar_loc.y() + range_min*sin(ray_angle);
    ray_line.p1.x() = lidar_loc.x() + range_max*cos(ray_angle);
    ray_line.p1.y() = lidar_loc.y() + range_max*sin(ray_angle);
    
    // Initialize variables for next loop
    Vector2f intersection_min;
    intersection_min.x() = lidar_loc.x() + range_max*cos(ray_angle);
    intersection_min.y() = lidar_loc.y() + range_max*sin(ray_angle);
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
  Particle& particle = *p_ptr;

  // Get predicted point cloud
  vector<Vector2f> predicted_cloud;
  GetPredictedPointCloud(particle.loc, particle.angle,
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

    Vector2f predicted_point = Map2BaseLink(predicted_cloud[i], particle.loc, particle.angle);
    float predicted_range = (predicted_point - Vector2f(0.2, 0)).norm();

    // New implementation of piecewise function of d_short and d_long
    float range_diff = ranges[i] - predicted_range;
    // if (range_diff < d_min_ or range_diff > d_max_){
    //   particle.log_weight -= 1e10;  // corresponds to a weight of 0
    //   return;
    // }
    range_diff = std::min(range_diff, d_long_);
    range_diff = std::max(range_diff,-d_short_);

    log_error_sum += -Sq(range_diff) / var_obs_;

    laser_angle += angle_diff;
  }

  particle.log_weight += log_error_sum;
}

// Done by Alex
void ParticleFilter::Resample() {
  // Check whether particles have been initialized
  if (particles_.empty()) return;

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

  if (division_size == 0) return;

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
  Resample();
}

// InProgress by Connor
// there is most recent estimate of location and then there are particles
// we use information about particles to update most recent estimate of location
// I think for loop actually needs to go at start fo this function.
// but then are we updating the state based on noise applied to all the particles? 
// how do we average that out?
void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  // prev_odom_loc_ = odom_loc;
  // prev_odom_angle_ = odom_angle;
  // for (auto &particle : particles_){
  //   //not sure ho
  //   if (particles_.empty()) 
  //   {
  //     //get most recent particle
  //     //i know this isn't correct use of pointer
  //     //and not even really sure which particle i am getting,
  //     //is it initialized or am i getting this afte resample?
  //     prev_odom_loc_ = &particle.loc;
  //     prev_odom_angle_ = &particle.angle
  //   }
  //   //get current pose (location and angle) of particle (pre-noise applied to it)
  //   //no idea what this & thingee is doing. i hate &s
  //   const Vector2f& odom_loc_ = &particle.loc;
  //   const float odom_angle_ = &particle.angle;
  //   const Vector2f& odom_trans_diff = (odom_loc_ - prev_odom_loc_).norm();
  //   const float angle_diff = std::abs(odom_angle_ - prev_odom_angle_);
  //   //apply noise to pose of particle
  //   UpdateParticleLocation(odom_trans_diff,angle_diff, &particle);
  // }
}

// TODO by Connor
void ParticleFilter::UpdateParticleLocation(Vector2f odom_trans_diff, float dtheta_odom, Particle* p_ptr)
{
  // Use the motion model to update each particle's location
  // This function will probably be called in the ObserveOdometry callback
  // You can update the particle location directly by modifying the particle variable
  // defined above since it was passed by reference (using the "&" symbol).
  // this particle passed by reference comes from ObserveLaser for loop
  // and is modified by Update function similar to how it is being modified here
  // but this occurs at every timestep

  //noise constants to tune
  // float k1 = 0.05;
  // float k2 = 0.025;
  // float k3 = 0.01;
  // float k4 = 0.05;
  
  // Particle& particle = *p_ptr;

  // //should the mean b
  // //is this how it should be, the meant is the same but the standard deviation, sigma, changes based on k constants
  // float eps_x = rng_.Gaussian(0.0,k1*odom_trans_diff + k2*dtheta_odom);
  // // future improvements wll use different constants for x and y to account for difference in slipping likelihood
  // float eps_y = rng_.Gaussian(0.0,k1*odom_trans_diff + k2*dtheta_odom);
  // float eps_angle = rng_.Gaussian(0.0,k3*odom_trans_diff + k4*dtheta_odom);
  // particle.loc += odom_trans_diff + Vector2f(eps_x,eps_y);
  // particle.angle += dtheta_odom + eps_angle;

  // // STARTER CODE that we can delete later on
  // // You will need to use the Gaussian random number generator provided. For
  // // example, to generate a random number from a Gaussian with mean 0, and
  // // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);
}

// Done by Mark
// Called by InitCallback in particle_filter_main
void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  particles_.clear(); // Need to get rid of particles from previous inits
  map_.Load("maps/" + map_file + ".txt"); // from Piazza
  cout << "Initialized " << map_file << " with " << map_.lines.size() << " lines!" << endl;

  // Make initial guesses (particles) based on a Gaussian distribution about initial placement
  for (size_t i = 0; i < FLAGS_num_particles; i++){
    Particle particle_init;
    particle_init.loc.x() = rng_.Gaussian(loc.x(), 0.25);  // std_dev of 0.25m, to be tuned
    particle_init.loc.y() = rng_.Gaussian(loc.y(), 0.25);  // std_dev of 0.25m, to be tuned
    particle_init.angle   = rng_.Gaussian(angle, M_PI/6);  // std_dev of 30deg, to be tuned
    particle_init.log_weight = 0;
    particles_.push_back(particle_init);
  }
}

// Done by Mark, untested
// Called by OdometryCallback in particle_filter_main
void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them.

  // Just do weighted average of loc and angle
  Vector2f weighted_loc_sum(0.0, 0.0);
  float weighted_angle_sum = 0.0;
  float weight_sum = 0.0;
  for (auto &particle : particles_)
  {
    weighted_loc_sum += particle.loc * particle.log_weight;
    weighted_angle_sum += particle.angle * particle.log_weight;
    weight_sum += particle.log_weight;
  }
  loc = weighted_loc_sum / weight_sum;
  angle = weighted_angle_sum / weight_sum;
}

// Helper function to convert from map to base_link, untested
Vector2f ParticleFilter::Map2BaseLink(const Vector2f& point, const Vector2f& loc, const float angle){
  Eigen::Rotation2Df R_inv(-angle); // negative of angle should be the same as transpose or inverse, right?
  Vector2f lidar_reading = R_inv*(point-loc); // transformation to lidar frame
  Vector2f lidar_offset(0.2, 0);
  return lidar_reading - lidar_offset; // transformation to base_link frame
}

}  // namespace particle_filter
