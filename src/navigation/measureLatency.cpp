#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"

#include <vector>
#include <fstream>

namespace {
	std::vector<double> x_dot_command_;
	std::vector<double> x_dot_odom_;
	std::vector<double> command_time_;
	std::vector<double> odom_time_;
	ros::Time start_time_;
}

void writeData()
{
	std::ofstream OutFile("calibration_readings.csv");

	OutFile << "Vx (Odom), Time (Odom), Vx (Command), Time (Command)\n";

	unsigned int max_size = std::max({x_dot_odom_.size(), x_dot_command_.size(), command_time_.size(), odom_time_.size()});
	for (unsigned int i = 0; i < max_size; i++)
	{
		// Record Odometry Velocity Readings
		if (i < x_dot_odom_.size()){
			OutFile << x_dot_odom_.at(i);
		}else{
			OutFile << "\t";
		}

		if (i < odom_time_.size()){
			OutFile << ",\t" << odom_time_.at(i);
		}else{
			OutFile << ",\t";
		}

		if (i < x_dot_command_.size()){
			OutFile << ",\t" << x_dot_command_.at(i);
		}else{
			OutFile << ",\t";
		}

		if (i < command_time_.size()){
			OutFile << ",\t" << command_time_.at(i);
		}else{
			OutFile << ",\t";
		}

		OutFile << "\n";
	}
}

void recordOdom(const nav_msgs::OdometryConstPtr &msg)
{
	x_dot_odom_.push_back(msg->twist.twist.linear.x);
	odom_time_.push_back((msg->header.stamp - start_time_).toSec());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "latency_measurement_node");
	ros::NodeHandle nh;
	ros::Rate r(100);

	ros::Subscriber odom_sub = nh.subscribe("/odom", 10, &recordOdom);
	ros::Publisher command_pub = nh.advertise<amrl_msgs::AckermannCurvatureDriveMsg>("/ackermann_curvature_drive", 1);

	amrl_msgs::AckermannCurvatureDriveMsg command_msg;

	x_dot_odom_.reserve(1000000000);
	x_dot_command_.reserve(1000000000);
	command_time_.reserve(1000000000);
	odom_time_.reserve(1000000000);

	start_time_ = ros::Time::now();
	while (ros::ok())
	{
		ros::Time now = ros::Time::now();
		double now_sec = (now - start_time_).toSec();

		command_msg.header.seq++; 
		command_msg.header.stamp = now;

		command_msg.velocity = 0.5*sin(now_sec); 	// 50 cm movement at 1 rad/s
		command_pub.publish(command_msg);

		command_time_.push_back(now_sec);
		x_dot_command_.push_back(command_msg.velocity);

		ros::spinOnce();
		r.sleep();
	}

	writeData();

	return 0;
}