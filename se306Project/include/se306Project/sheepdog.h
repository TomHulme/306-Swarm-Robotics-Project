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

float prevclosestRange = 0;
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double tz;
double theta;
double prevpx;
double prevpy;
bool isRotate;
int sheepNum = 0;
enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
ros::Publisher sheepdogPosPub;
ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
enum FSM fsm; // Finite state machine for the random walk algorithm

ros::Time rotateStartTime; // Start time of the rotation
ros::Time rotateEndTime;
ros::Duration rotateDuration; // Duration of the rotation
ros::Subscriber StageOdo_sub;

ros::NodeHandle node_;
laser_geometry::LaserProjection projector_;
tf::TransformListener tfListener_;

ros::Publisher point_cloud_publisher_;
int checkcount=0;




class sheepdogNode {
	public:	
	sheepdogNode(int);
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
