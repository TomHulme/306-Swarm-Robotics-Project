//============================================================================
// Name        : Field.cpp
// Author      : John Lambert
// Date		   : 7/08/2013
// Version     : 
// Description : Class file of the node that will represent a field in our farm
//	simulation.
//============================================================================

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>

//-------------------------------------------------------------------------------------
enum SoilQuality {
	FERTILE, NORMAL, ARID
};
class FieldNode {
	static const int sunData[12];

public:
	FieldNode(int, double, double);
	void *fieldRun();
	void rosSetup(int, char**);
	//parameters of the field
	int fieldNum;
	double sizeX, sizeY;
	SoilQuality soil;
	double rain;
	int sunLight;
	bool morning;
	int sunIndex;
	void setRain();
	void setSoil();
	void setSunLight();
	char* getSoilType();
};

char* FieldNode::getSoilType() {
	if (this->soil == SoilQuality(FERTILE)) {
		return "Fertile";
	} else if (this->soil == SoilQuality(NORMAL)) {
		return "Normal.";
	} else {
		return "Arid.";
	}
}

FieldNode::FieldNode(int number, double x, double y) {
	fieldNum = number;
	sizeX = x;
	sizeY = y;
	soil = NORMAL;
	rain = 50;
	sunLight = 0;
	morning = false;
	sunIndex = 0;
}

void FieldNode::setRain() {
	//set rain to random number out of 100 (0 - 99)
	rain = rand() % 100;
}

void FieldNode::setSoil() {
	switch (soil) {
	case FERTILE:
		if (rain < 2) { //2% chance
			soil = ARID;
		} else if (rain < 80) { //78% chance
			soil = NORMAL;
		} else { //20% chance
			soil = FERTILE;
		}
		break;
	case NORMAL:
		if (rain < 20) { //20% chance
			soil = ARID;
		} else if (rain < 80) { //60% chance
			soil = NORMAL;
		} else { //20% chance
			soil = FERTILE;
		}
		break;
	case ARID:
		if (rain < 20) { //2% chance
			soil = ARID;
		} else if (rain < 98) { //78% chance
			soil = NORMAL;
		} else { //20% chance
			soil = FERTILE;
		}
		break;
	}
}

void FieldNode::setSunLight() {
	//change whether its morning or not
	if (sunIndex == 0 || sunIndex == 11) {
		if (morning) {
			morning = false;
		} else {
			morning = true;
		}
	}
	//set Sunlight based on the sunData array
	int increment;
	if (morning) { //if its am, set increment to +1
		increment = 1;
	} else {	//else set it to -1
		increment = -1;
	}

	sunLight = sunData[sunIndex];
	sunIndex = sunIndex + increment;
}

void FieldNode::rosSetup(int argc, char **argv) {
	//Initate ros with name determined by field number.
	std::string name;
	std::ostringstream convert;
	convert << fieldNum;
	name = "Field" + convert.str();

	ros::init(argc, argv, name);

	ros::NodeHandle n;

	//Create publisher to determine if its functioning
	ros::Publisher FieldNode_inc_msg = n.advertise < nav_msgs::Odometry
			> (name, 1000);

	ros::Rate loop_rate(10);

	//ensures the field is only changed every second
	int count = 0;
	while (ros::ok()) {
		if (count == 10) {
			this->setRain();
			this->setSoil();
			this->setSunLight();
			count = 0;
			ROS_INFO("Current sunlight is %d", this->sunLight);
			ROS_INFO("Current soil type is %s", this->getSoilType());
			ROS_INFO("Current rain is at %f", this->rain);
		}
		++count;
		ros::spinOnce();
		loop_rate.sleep();
	}
}
//--------------------------------------------------------------------------------------

//definition of sunData
const int FieldNode::sunData[12] = { 0, 0, 0, 10, 20, 40, 70, 90, 100, 100, 100,
		100 };

//create field objects. assign them node handlers. start run method through thread
int main(int argc, char **argv) {
	FieldNode field0 = FieldNode(1, 5, 5);

	field0.rosSetup(argc, argv);
}

