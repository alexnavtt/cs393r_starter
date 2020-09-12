#ifndef LATENCY_COMPENSATOR_HH
#define LATENCY_COMPENSATOR_HH

#include <list>
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose2D.h"

struct state2D{
  float x, y, theta;
  float vx, vy, omega;
};

class LatencyCompensator {
public:
  LatencyCompensator(float actuation_delay, float observation_delay, float delta_t);

  // Setters and Getters
  double getActuationDelay();
  void  setActuationDelay(float delay);
  double getObservationDelay();
  void  setObservationDelay(float delay);
  double getSystemDelay();

  void recordNewInput(float x_dot, float y_dot, float omega);
  void recordObservation(float x, float y, float theta);
  state2D predictedState();

private:
  std::list< std::array<double,4> > recordedInputs_; // Record of past inputs in the form (curvature, speed, timestamp)
  double actuation_delay_;                           // Robot actuation delay (seconds)
  double observation_delay_;                         // Sensor observation delay (seconds)
  double last_observation_time_;                     // Timestamp for when the last sensor state came in 
  double delta_t_;                                   // Duration of a control loop for the system being compensated

  state2D state_;                     // Last recorded state of the robot
};

#endif
