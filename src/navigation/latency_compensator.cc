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
double LatencyCompensator::getActuationDelay() {return actuation_delay_;}
void  LatencyCompensator::setActuationDelay(float delay) {actuation_delay_ = delay;} 

// Observation Delay Setter and Getter
double LatencyCompensator::getObservationDelay() {return observation_delay_;}
void  LatencyCompensator::setObservationDelay(float delay) {observation_delay_ = delay;}

// System Delay Getter
double LatencyCompensator::getSystemDelay() {return actuation_delay_ + observation_delay_;}

// Note when an observation is taken
void LatencyCompensator::recordObservation(float x, float y, float theta){
	state_.x = x;
	state_.y = y;
	state_.theta = theta;
	last_observation_time_ = ros::Time::now().toSec() - observation_delay_;
}

// Note and record when a command was inputted
void LatencyCompensator::recordNewInput(float x_dot, float y_dot, float omega) {
	recordedInputs_.push_back(std::array<double,4>({double(x_dot), double(y_dot), double(omega), ros::Time::now().toSec()}));
	state_.vx = x_dot;
	state_.vy = y_dot;
	state_.omega = omega;
}

state2D LatencyCompensator::predictedState()
{
	// Do nothing if no observations have been made or if delays are exactly 0
	if (last_observation_time_ < 0 or (actuation_delay_ == 0 && observation_delay_ == 0)) return state_; 

	// Initialize the output
	state2D predicted_state = state_;

	// Determine how long ago inputs went stale (i.e. they occured before the latest observation)
	double cutoff_time = last_observation_time_ - actuation_delay_;

	// Iterate through inputs and forward predict the current state
	for (auto input_stamped = recordedInputs_.begin(); input_stamped != recordedInputs_.end();)
	{
		// Erase any input that's too old to be important (this assumes that the list is sorted chronologically)
		if (input_stamped->at(3) <= cutoff_time) 
		{
			input_stamped = recordedInputs_.erase(input_stamped); 
			continue;
		}
		
		// Use recent inputs to forward predict state of the robot
		else
		{			
			// Predict State
			predicted_state.x 	  += input_stamped->at(0) * delta_t_;
			predicted_state.y 	  += input_stamped->at(1) * delta_t_;
			predicted_state.theta += input_stamped->at(2) * delta_t_;
			input_stamped++;
		}
	}

	// For if all inputs were deleted, then the robot's velocity is assumed to be the last commanded velocity
	if ( recordedInputs_.empty() ) return predicted_state;

	// Assuming that the car will instantaneously go to whatever speed we command it to
	auto current_input = recordedInputs_.begin();
	predicted_state.vx = current_input->at(0);
	predicted_state.vy = current_input->at(1);
	predicted_state.omega = current_input->at(2);

	return predicted_state;
}