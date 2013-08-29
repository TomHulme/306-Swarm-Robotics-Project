#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "se306Project/FieldMsg.h"
#include "se306Project/GrassPosMsg.h"
#include "geometry_msgs/Twist.h"
#include "se306Project/SheepEatMsg.h"

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
	void sheepEatingCallback(se306Project::SheepEatMsg);

	int grassNum;
	int robotNum;
	int fieldNum;
	double grassX;
	double grassY;
	double grassZ;
	//parameters that need to be used eventually
	double growthRate; // %/s
	double decayRate; // %/s
	double overallRate; // rate of grass growth/decay %/s
	double grassHeight; // growth of grass 0-100%
	std::string soilQuality;
	int sunLight;
	int nextZ;

protected:
	ros::Subscriber StageOdo_sub;
	ros::Publisher commandPub;
	ros::Subscriber FieldNode_sub;
	ros::Publisher grassPosPub;
	ros::Subscriber sheepEatGrassSub;
};

GrassNode::GrassNode() {
	grassNum = 0;
	robotNum = 0;
	fieldNum = 0;
	grassX = 0;
	grassY = 0;
	grassZ = 0;
	growthRate = 0;
	decayRate = 0;
	overallRate = 0;
	grassHeight = 100;
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
	ROS_INFO("growth rate: %f", growthRate);
	ROS_INFO("eaten amount: %f", decayRate);
	overallRate = growthRate - decayRate;
	ROS_INFO("total growth: %f", overallRate);
	overallRate = -overallRate*0.1; // growing is negative
	ROS_INFO("absolute growth: %f", overallRate);
	ROS_INFO("Grass Z: %f", grassZ);
	// Calculate growth
	if (grassZ > -0.5 && grassZ <= 0) {
		grassHeight = 100;
		if (overallRate<0) {
			overallRate = 0; // stop growing above 100%
		}
	} else if (grassZ <= -0.5 && grassZ >= -1) {
		grassHeight = 0;
		if (overallRate>0) {
			overallRate = 0; // stop decaying below 0%
		}
	}
	decayRate = 0;
	// Display netGrowth
	std::ostringstream convertG;
	convertG << grassHeight;
	std::string grassHeightStr = convertG.str();
	ROS_INFO_STREAM("Current Height: " + grassHeightStr);

	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.angular.z = overallRate;
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
	grassPosPub = n.advertise<se306Project::GrassPosMsg>("grass/info", 1000);
	sheepEatGrassSub = n.subscribe<se306Project::SheepEatMsg>("grass/eaten", 1000, &GrassNode::sheepEatingCallback,this);

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
		growthRate = 0.5; // %/s
	} else if (soilQuality == "Normal") {
		growthRate = 1; // %/s
	} else {
		growthRate = 2; // %/s
	}
	
	growthRate = ((50+(double)sunLight)/100)*growthRate; // %/s
}

void GrassNode::sheepEatingCallback(se306Project::SheepEatMsg msg) {
	if (msg.grassNum == grassNum) {
		decayRate += msg.eatAmount;
	}
}

void GrassNode::spin() {
	ros::Rate rate(10); // 10 Hz
	while (ros::ok()) {
		this->grassGrow();

		se306Project::GrassPosMsg msg;

		msg.grassNum = this->grassNum;
		msg.x = this->grassX;
		msg.y = this->grassY;
		msg.grassHeight = this->grassHeight;

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
