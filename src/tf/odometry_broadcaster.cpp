#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <eigen3/Eigen/Dense>

#include "amrl_msgs/Localization2DMsg.h"

class Data{
public:
	Eigen::Vector2f odom_loc;
	float odom_angle;
	Eigen::Vector2f robot_loc;
	float robot_angle;

	tf2_ros::TransformBroadcaster tf_broadcaster;
	geometry_msgs::TransformStamped odom_trans;

	void updateOdom(const nav_msgs::Odometry& msg){
		odom_trans.header.seq++;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id  = "base_link";

		odom_trans.transform.translation.x = msg.pose.pose.position.x;
		odom_trans.transform.translation.y = msg.pose.pose.position.y;
		odom_trans.transform.translation.z = msg.pose.pose.position.z;

		odom_trans.transform.rotation.w = msg.pose.pose.orientation.w;
		odom_trans.transform.rotation.x = msg.pose.pose.orientation.x;
		odom_trans.transform.rotation.y = msg.pose.pose.orientation.y;
		odom_trans.transform.rotation.z = msg.pose.pose.orientation.z;

		tf_broadcaster.sendTransform(odom_trans);
	}

	void updateLocalization(const amrl_msgs::Localization2DMsg &msg){

	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle nh;

	Data DATA;

	ros::Subscriber odom_sub = nh.subscribe("odom", 1, &Data::updateOdom, &DATA);
	ros::Subscriber loc_sub  = nh.subscribe("Localization", 1, &Data::updateLocalization, &DATA);

	geometry_msgs::TransformStamped baseLink2Lidar;
	baseLink2Lidar.header.stamp = ros::Time::now();
	baseLink2Lidar.header.frame_id = "odom";
	baseLink2Lidar.child_frame_id = "laser";

	baseLink2Lidar.transform.translation.x = 0.2;
	baseLink2Lidar.transform.translation.z = 0.1;
	baseLink2Lidar.transform.rotation.w    = 1.0;	// Identity quaternion

	tf2_ros::StaticTransformBroadcaster tf_static_broadcaster;
	tf_static_broadcaster.sendTransform(baseLink2Lidar);

	ros::spin();
}
