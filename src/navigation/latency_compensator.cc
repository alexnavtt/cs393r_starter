#include "latency_compensator.h"

  /////////////////////////
 // Latency Compensator //
/////////////////////////
LatencyCompensator::LatencyCompensator(float actuation_delay, float observation_delay, float dt)
{
	actuation_delay_   = actuation_delay;
	observation_delay_ = observation_delay;
	system_delay_ = actuation_delay + observation_delay;
	last_observation_time_ = -1.0;
	delta_t_ = dt;
}

void LatencyCompensator::recordObservation()
{
	last_observation_time_ = ros::Time::now().toSec();
}

void LatencyCompensator::recordNewInput(float x_dot, float y_dot, float omega)
{
	recordedInputs_.push_back(std::array<float,4>({x_dot, y_dot, omega, float(ros::Time::now().toSec())}));
}

geometry_msgs::Pose2D LatencyCompensator::predictedState(Eigen::Vector2f Loc, float theta)
{
	geometry_msgs::Pose2D predicted_state;
	predicted_state.x = Loc[0];
	predicted_state.y = Loc[1];
	predicted_state.theta = theta;

	float cutoff_time = last_observation_time_ - actuation_delay_;

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