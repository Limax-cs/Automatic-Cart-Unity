#include <ros/ros.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "Hello World");
	ros::NodeHandle nh;
	ros::Rate loop_rate(2);
	
	while (ros::ok())
	{
		ROS_INFO("Hello World");
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
