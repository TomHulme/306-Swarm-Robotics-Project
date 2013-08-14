#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "Field0");
	ros::NodeHandle n;
	ros::Publisher soilDigest_pub = n.advertise<se306Project::soilQualityMessage>("Soil_state", 1000);
	
	while (ros::ok())
	{
		se306Project::soilQualityMessage msg;
		msg.quality = 2; //TODO: associate with enum. TODO: non-hard code this variable
		soilDigest_pub.publish(msg);
	}
	
	return 0;
};