//============================================================================
// Name        : Field.cpp
// Author      : John Lambert
// Date	       : 16/08/2013
// Version     : 
// Description : Field Class containing all methods for the Field node
//============================================================================

#include <se306Project/Field.h>

char* FieldNode::getSoilType() {
	if (this->soil == SoilQuality(FERTILE)) {
		return "Fertile";
	} else if (this->soil == SoilQuality(NORMAL)) {
		return "Normal";
	} else {
		return "Arid";
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

//Changes the rain value
void FieldNode::setRain() {
	//set rain to random number out of 100 (0 - 99)
	srand(time(NULL) + fieldNum * sunLight);
	rain = rand() % 100;
}

//Changes the soil parameter
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
		if (rain < 20) { //20% chance
			soil = ARID;
		} else if (rain < 98) { //78% chance
			soil = NORMAL;
		} else { //2% chance
			soil = FERTILE;
		}
		break;
	}
}

//Changes the sunlight value
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
	ros::Publisher FieldNode_soil_msg = n.advertise < se306Project::FieldMsg
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

			//Publish field quality on a topic
			se306Project::FieldMsg msg;

			msg.sunLight = this->sunLight;
			msg.quality = this->getSoilType();

			FieldNode_soil_msg.publish(msg);

			//Used to check parameters of field while running.
			ROS_INFO("Current sunlight is %d", this->sunLight);
			ROS_INFO("Current soil type is %s.", this->getSoilType());
			ROS_INFO("Current rain is at %f", this->rain);
		}
		++count;
		ros::spinOnce();
		loop_rate.sleep();
	}
}
