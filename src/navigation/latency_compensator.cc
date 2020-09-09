#include "latency_compensator.h"

  /////////////////////////
 // Latency Compensator //
/////////////////////////
LatencyCompensator::LatencyCompensator(float actuation_delay, float observation_delay, float dt)
{
	actuation_delay_   	= actuation_delay;
	observation_delay_ 	= observation_delay;
	delta_t_ 			= dt;

	last_observation_time_ = -1.0;
}

// Actuation Delay Setter and Getter
float LatencyCompensator::getActuationDelay() {return actuation_delay_;}
void  LatencyCompensator::setActuationDelay(float delay) {actuation_delay_ = delay;} 

// Observation Delay Setter and Getter
float LatencyCompensator::getObservationDelay() {return observation_delay_;}
void  LatencyCompensator::setObservationDelay(float delay) {observation_delay_ = delay;}

// System Delay Getter
float LatencyCompensator::getSystemDelay() {return actuation_delay_ + observation_delay_;}

// Note when an observation is taken
void LatencyCompensator::recordObservation(float x, float y, float theta){
	state_.x = x;
	state_.y = y;
	state_.theta = theta;
	last_observation_time_ = ros::Time::now().toSec() - observation_delay_;
}

// Note and record when a command was inputted
void LatencyCompensator::recordNewInput(float x_dot, float y_dot, float omega) {
	recordedInputs_.push_back(std::array<float,4>({x_dot, y_dot, omega, float(ros::Time::now().toSec())}));
}

geometry_msgs::Pose2D LatencyCompensator::predictedState()
{
	// Do nothing if no observations have been made
	if (last_observation_time_ < 0) return state_; 

	// Initialize the output
	geometry_msgs::Pose2D predicted_state = state_;

	// Determine how long ago inputs went stale (i.e. they occured before the latest observation)
	float cutoff_time = last_observation_time_ - actuation_delay_;

	// Iterate through inputs and forward predict the current state
	for (const auto &input_stamped : recordedInputs_)
	{
		// Erase any input that's too old to be important (this assumes that the list is sorted chronologically)
		if (input_stamped.at(3) < cutoff_time) recordedInputs_.pop_front(); 
		
		// Use recent inputs to forward predict state of the robot
		else
		{			
			// Predict State
			predicted_state.x 	  += input_stamped.at(0) * delta_t_;
			predicted_state.y 	  += input_stamped.at(1) * delta_t_;
			predicted_state.theta += input_stamped.at(2) * delta_t_;
		}
	}

	return predicted_state;
}