//============================================================================
// Name        : sheepdog.h
// Author      : Tom Hulme
// Date		   : 28/08/2013
// Version     : 
// Description : 
//============================================================================

#ifndef SHEEPDOG_H_
#define SHEEPDOG_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <cstdlib>
#include <ctime>



class sheepdogNode {
	public:	
	sheepdogNode(int, ros::NodeHandle&);
	void rosSetup(int, char**);
	
	void StageOdom_callback(nav_msgs::Odometry);
	void move(double, double);
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr&);
	void chaseSheepCallback(geometry_msgs::Pose2D);
	void spin();
	
	

	
	//--------------------
	sheepdogNode() { } //Default constructor
};
#endif /* SHEEPDOG_H_ */
