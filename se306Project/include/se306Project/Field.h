//============================================================================
// Name        : Field.h
// Author      : John Lambert
// Date		   : 16/08/2013
// Version     : 
// Description : 
//============================================================================

#ifndef FIELD_H_
#define FIELD_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <time.h>

enum SoilQuality {
	NORMAL, FERTILE, ARID
};

class FieldNode {
	static const int sunData[12];

public:
	FieldNode(int, double, double);
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
	//--------------------
	FieldNode() { } //Default constructor
};
#endif /* FIELD_H_ */
