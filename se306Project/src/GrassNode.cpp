#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "se306Project/FieldMsg.h"
#include "se306Project/GrassPosMsg.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <ctime>

class GrassNode {

public:
	GrassNode();
	GrassNode(int, int, int);
	void grassGrow();
	void rosSetup(int, char**);
	void spin();
	void stageOdom_callback(nav_msgs::Odometry);
	void fieldNode_callback(se306Project::FieldMsg);

	int grassNum;
	int robotNum;
	int fieldNum;
	double grassX;
	double grassY;
	double grassZ;
	//parameters that need to be used eventually
	double growth; // %/s
	double decay; // %/s
	double netGrowth;
	std::string soilQuality;
	int sunLight;
	int nextZ;

protected:
	ros::Subscriber StageOdo_sub;
	ros::Publisher commandPub;
	ros::Subscriber FieldNode_sub;
	ros::Publisher grassPosPub;
};

GrassNode::GrassNode() {
	grassNum = 0;
	robotNum = 0;
	fieldNum = 0;
	grassX = 0;
	grassY = 0;
	grassZ = 0;
	growth = 0;
	decay = 0;
	netGrowth = 0;
	soilQuality = "";
	sunLight = 0;
	nextZ = 0;
}

GrassNode::GrassNode (int grass, int robot, int field) {
	GrassNode();
	this->grassNum = grass;
	this->robotNum = robot;
	this->fieldNum = field;
}	

void GrassNode::grassGrow(){
	
	decay = growth + 1;
	netGrowth = growth - decay;	// %/s
	
	// % to angle conversion	
	netGrowth = -netGrowth*0.1;	// angle/
	nextZ = grassZ + netGrowth;

	// Display grassZ
	std::ostringstream convertZ;
	convertZ << grassZ;
	std::string grassZStr = convertZ.str();
	ROS_INFO_STREAM("grassZ: " + grassZStr);	

	if (grassZ > -0.5 && grassZ < 0) {
		netGrowth = 0 - grassZ;
	} else if (grassZ <= -0.5 && grassZ > -1) {
		netGrowth = 0.999999999999999999999999999999999999999989999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999 + grassZ;
	}

	// Display netGrowth
	std::ostringstream convertG;
	convertG << netGrowth;
	std::string netGrowthStr = convertG.str();
	ROS_INFO_STREAM("netGrowth: " + netGrowthStr);

	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.angular.z = netGrowth;
	commandPub.publish(msg);
}

void GrassNode::rosSetup(int argc, char **argv) {
/**	
	std::ostringstream convertG;
	std::ostringstream convertR;
	std::ostringstream convertF;	
	ros::init(argc, argv, "grass", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	// Get grass number and robot number
	n.getParam("grassNum", grassNum);
	n.getParam("robotNum", robotNum);
	n.getParam("fieldNum", fieldNum);
	convertG << grassNum;
	convertR << robotNum;
	convertF << fieldNum;
	// Convert these into strings
	std::string g = "grass_" + convertG.str();
	ROS_INFO_STREAM(g);
	std::string r = "robot_" + convertR.str();
	ROS_INFO_STREAM(r);
	std::string f = "field_" + convertF.str();
	ROS_INFO_STREAM(f);
**/
	//Initate ros with name determined by field number.
	std::string name;
	std::ostringstream convert;
	std::ostringstream convertG;
	std::ostringstream convertR;
	std::ostringstream convertF;	
	convert << this->grassNum;
	name = "Grass" + convert.str();

	ros::init(argc, argv, name);

	ros::NodeHandle n;

	convertG << this->grassNum;
	convertR << this->robotNum;
	convertF << this->fieldNum;
	// Convert these into strings
	std::string g = "grass_" + convertG.str();
	ROS_INFO_STREAM(g);
	std::string r = "robot_" + convertR.str();
	ROS_INFO_STREAM(r);
	std::string f = "Field" + convertF.str();
	ROS_INFO_STREAM(f);

	//initialise the talkies
	StageOdo_sub = n.subscribe<nav_msgs::Odometry>(r + "/odom",1000, &GrassNode::stageOdom_callback,this);
	commandPub = n.advertise<geometry_msgs::Twist>(r + "/cmd_vel",1000);
	//commandPub = n.advertise<nav_msgs::Odometry>(r + "/odom",1000);
	FieldNode_sub = n.subscribe<se306Project::FieldMsg>(f,1000, &GrassNode::fieldNode_callback, this);
	grassPosPub = n.advertise<se306Project::GrassPosMsg>("grassPos", 1000);
	//sheepdogPosSub = n.subscribe<std_msgs::String>("sheepdog_position",1000, &SheepNode::sheepdogDangerCallback,this);

	//: talk to the grass, and the field?

	GrassNode::spin();
	
}

void GrassNode::stageOdom_callback(nav_msgs::Odometry msg) {
	grassX = 0 + msg.pose.pose.position.x;
	grassY = 0 + msg.pose.pose.position.y;	
	grassZ = 0 + msg.pose.pose.orientation.z;
}

void GrassNode::fieldNode_callback(se306Project::FieldMsg msg) {
	soilQuality = msg.quality;
	sunLight = msg.sunLight;
	
	ROS_INFO_STREAM("soilQuality:");
	ROS_INFO_STREAM(soilQuality); // Prints the current z value of the grass	

	if (soilQuality == "Arid") {
		growth = 0.25; // %/s
	} else if (soilQuality == "Normal") {
		growth = 0.5; // %/s
	} else {
		growth = 1; // %/s
	}
	
	growth = ((50+(double)sunLight)/100)*growth; // %/s == 1.8*growth
}

void GrassNode::spin() {
	ros::Rate rate(10); // 10 Hz
	while (ros::ok()) {
		this->grassGrow();

		se306Project::GrassPosMsg msg;

		msg.grassNum = this->grassNum;
		msg.x = this->grassX;
		msg.y = this->grassY;
		
		int fGrowth = 100-(100*this->grassZ);

		msg.growth = fGrowth;

		grassPosPub.publish(msg);

		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc, char **argv) {

	int grassNum = atoi(argv[1]);
	int robotNum = atoi(argv[2]);
	int fieldNum = atoi(argv[3]);
	
	GrassNode grass = GrassNode(grassNum, robotNum, fieldNum);

	grass.rosSetup(argc, argv);

}
