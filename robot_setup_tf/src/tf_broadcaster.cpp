
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

using namespace std;
#include "cmath"



int main(int argc, char **argv) {

	ros::init(argc, argv, "robot_tf_publisher");

	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;

	float x ,y ,z,w;

	float pitch, roll, yaw;
	// rotate by x ,y ,z axis
	pitch = 0;

	roll = (45 * 3.14) / 180;

	yaw = (270 * 3.14) / 180;

	x = (sin(pitch / 2) * cos(roll / 2) * cos(yaw / 2) + cos(pitch / 2) * sin(roll / 2) * sin(yaw / 2));
	y = cos(pitch / 2) * sin(roll / 2) * cos(yaw / 2) - sin(pitch / 2) * cos(roll / 2) * sin(yaw / 2);
	z = cos(pitch / 2) * cos(roll / 2) * sin(yaw / 2) - sin(pitch / 2) * sin(roll / 2) * cos(yaw / 2);
	w = cos(pitch / 2) * cos(roll / 2) * cos(yaw / 2) + sin(pitch / 2) * sin(roll / 2) * sin(yaw / 2);

	// ROS_INFO("x = %f",pitch);
	// ROS_INFO("y = %f",roll);
	// ROS_INFO("z = %f",yaw);
	ROS_INFO("x = %f",x);
	ROS_INFO("y = %f",y);
	ROS_INFO("z = %f",z);
	ROS_INFO("w = %f",w);

	while (n.ok()) {

		broadcaster.sendTransform(

				tf::StampedTransform(

						tf::Transform(tf::Quaternion(x, y, z, w), tf::Vector3(0.0, 15.0, 5.0)),

						ros::Time::now(),"base_link", "base_Camera"));

		r.sleep();

	}

}

