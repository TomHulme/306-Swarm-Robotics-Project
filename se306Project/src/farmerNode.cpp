//============================================================================
// Name        : farmerNode.cpp
// Author      : Tom Hulme
// Date		   : 29/08/2013
// Version     : 
// Description : Class file of the node that will represent a sheepdof in our 
//	farm simulation.
//============================================================================

#include <se306Project/farmer.h>

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>

//Creates a single field node based on inputs given
int main(int argc, char **argv) {
	int xi = 5;
	int yi = 2;
	farmerNode farmer = farmerNode(xi,yi);
	farmer.rosSetup(argc, argv);
}
