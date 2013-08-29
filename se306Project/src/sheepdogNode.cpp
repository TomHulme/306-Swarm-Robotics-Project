//============================================================================
// Name        : sheepdogNode.cpp
// Author      : Tom Hulme
// Date		   : 23/08/2013
// Version     : 
// Description : Class file of the node that will represent a sheepdof in our 
//	farm simulation.
//============================================================================

#include <se306Project/sheepdog.h>

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>


ros::NodeHandle n;

//Creates a single field node based on inputs given
int main(int argc, char **argv) {
	
	int sheepNum = atoi(argv[1]);
	sheepdogNode sheepdog = sheepdogNode(sheepNum, n);

	sheepdog.rosSetup(argc, argv);
}
