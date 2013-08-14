#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

double px;
double py;
double height;
bool isDead;


void GrowCallback(se306Project::soilQualityMessage msg)
{
	//change height depending on soil quality
	ROS_INFO_STREAM("Grass got soil quality " + msg->quality);
	if (height <= 0) 
	{
		isDead = true;
	}
	//grow
		
}

int main (int argc, char **argv)
{
	//set px, py, height
	isDead = false;
	ros::init(argc, argv, "Grass")
	ros::NodeHandle n;
	ros::Subscriber soilData_sub = n.subscribe<se306Project::soilQualityMessage>("Soil_state", 1000, GrowCallback);
	ros::spin();
	return 0;
}