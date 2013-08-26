#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <ctime>

class GrassNode {

public:
	GrassNode();
	void grassGrow();
	void rosSetup(int, char**);
	void spin();
	void stageOdom_callback(nav_msgs::Odometry);

	int grassNum;
	int robotNum;
	double grassZ;
	//parameters that need to be used eventually
	int growth; //0%-100%

protected:
	ros::Subscriber StageOdo_sub;

};

GrassNode::GrassNode() {
	grassNum = 0;
	grassZ = 0;
	growth = 100;
}

void GrassNode::grassGrow(){
	//subscribe to current position grass service
	//use service to eat grass? (say eating, grass sends back ok to keep eating, or stop eating.
	//if stop eating, change state to walking, publish GO to sheep_x/move (sheepMovePub)
	std::ostringstream convertZ;
	convertZ << grassZ;
	std::string grassZStr = convertZ.str();

	ROS_INFO_STREAM(grassZStr);
}

void GrassNode::rosSetup(int argc, char **argv) {
	
	std::ostringstream convertG;
	std::ostringstream convertR;
	ros::init(argc, argv, "grass", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	// Get grass number and robot number
	n.getParam("grassNum", grassNum);
	n.getParam("robotNum", robotNum);
	convertG << grassNum;
	convertR << robotNum;
	// Convert these into strings
	std::string g = "grass_" + convertG.str();
	ROS_INFO_STREAM(g);
	std::string r = "robot_" + convertR.str();
	ROS_INFO_STREAM(r);
	

	//initialise the talkies
	StageOdo_sub = nh.subscribe<nav_msgs::Odometry>(r + "/odom",1000, &GrassNode::stageOdom_callback,this);
	//sheepMovePub = nh.advertise<se306Project::SheepMoveMsg>("grass_" + convert.str()+ "/move", 1000);
	//sheepdogPosSub = nh.subscribe<std_msgs::String>("sheepdog_position",1000, &SheepNode::sheepdogDangerCallback,this);

	//TODO: talk to the grass, and the field?
	
	GrassNode::spin();
	
}

void GrassNode::stageOdom_callback(nav_msgs::Odometry msg) {	
	grassZ = 0 + msg.pose.pose.position.z;
}

void GrassNode::spin() {
	ros::Rate rate(10); 
	while (ros::ok()) {
		this->grassGrow();
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc, char **argv) {
	
	GrassNode grass = GrassNode();

	grass.rosSetup(argc, argv);

}
