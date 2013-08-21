//============================================================================
// Name        : FieldNode.cpp
// Author      : John Lambert
// Date		   : 7/08/2013
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

//create field objects. assign them node handlers. start run method through thread
int main(int argc, char **argv) {
	FieldNode field = FieldNode(0, (double)5, (double)5);

	field.rosSetup(argc, argv);

	return 0;
}

