//============================================================================
// Name        : farmer.h
// Author      : Tom Hulme
// Date		   : 28/08/2013
// Version     : 
// Description : Farmer Header File
//============================================================================

#ifndef FARMER_H_
#define FARMER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"


class farmerNode {
	public:	
	
	farmerNode(int,int);
	void rosSetup(int, char**);

	//velocity of the robot
	double linear_x;
	double angular_z;
	//pose of the robot
	double px;
	double py;
	double theta;
	double prevpx;
	double prevpy;

	
	void StageOdom_callback(nav_msgs::Odometry);
	void StageLaser_callback(sensor_msgs::LaserScan);
	
	//--------------------
	farmerNode() { } //Default constructor
};
#endif /* FARMER_H_ */
