//============================================================================
// Name        : FieldNode.cpp
// Author      : John Lambert
// Date		   : 23/08/2013
// Version     : 
// Description : Class file of the node that will represent a field in our farm
//	simulation.
//============================================================================

#include <se306Project/Field.h>

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>

//definition of sunData
const int FieldNode::sunData[12] = { 0, 0, 0, 10, 20, 40, 70, 90, 100, 100, 100,
		100 };

//Creates a single field node based on inputs given
int main(int argc, char **argv) {
	
	int fieldNum = atoi(argv[1]);
	double fieldX = (double)atoi(argv[2]);
	double fieldY = (double)atoi(argv[3]);

	FieldNode field = FieldNode(fieldNum, fieldX, fieldY);

	field.rosSetup(argc, argv);
}

