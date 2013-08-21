#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "se306Project/SheepMoveMsg.h"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <ctime> // Needed to seed random number generator with a time value

enum SheepState {
	WALKING, RUNNING, EATING
};

class SheepNode {
	
public:
	//setup methods
	SheepNode();//(int);
	void rosSetup(int, char**);
	void spin();	
	
	int sheepNum;
	SheepState currentState;
	int currX;
	int currY;
	//parameters that need to be used eventually
	int terror;
	int age;
	
//===movement related variables
	float prevclosestRange;
	double linear_x;
	double angular_z;
	//pose of the robot
	double px;
	double py;
	double theta;
	double prevpx;
	double prevpy;
	bool isRotate;
	int checkcount;
//===

	void sheepEat();
	void sheepWalk();
	
	//TODO: Sheep Danger sense
	void sheepDogDangerCallback(std_msgs::String);
	//TODO: Eating
	//void eatCallback();
	
	protected:
	ros::Publisher sheepMovePub;
	ros::Subscriber sheepdogPosSub;
};

void SheepNode::sheepDogDangerCallback(std_msgs::String sheepdogMsg) {
	std::string sheepdogPos (sheepdogMsg);
	std::string::size_type sz;
		
	float sdx = std::stof(sheepdogPos,&sz);
	float sdy = std::stof(sheepdogPos.substr(sz));
		
	//Calculate the difference in distance between the sheepdog(sdx)[std_msgs::String msg?] and sheep
	//closeRange=;
	xDistanceDiff = abs(sdx - px);
	yDistanceDiff = abs(sdy - py);

	//if sheepdog is near: level of terror is raised depending on the distance to the sheepdog. 
	//	eg. if it is 10 units away (or whatever distance seems appropriate), then terror is low, but exists. 
	//       if it is at 6 units away, terror goes up again, and sheep move away from the dog.
	//		 if it is at 3 units away or less, sheep runs away from dog.
	//if (xDistanceDiff<=20||yDistanceDiff<=20){
	//	scared(sdx,sdy);
	//}
}


void SheepNode::sheepEat(){
	//subscribe to current position grass service
	//use service to eat grass? (say eating, grass sends back ok to keep eating, or stop eating.
		//if stop eating, change state to walking, publish GO to sheep_x/move (sheepMovePub) 
}

void SheepNode::sheepWalk() {
	//check if there is edible grass here
		//then publish STOP to sheep_x/move (sheepMovePub) 
	//else send move message
}

void SheepNode::spin() {
	//do things depending on SheepState
	ros::Rate rate(10); 
	while (ros::ok()) {
		//bool stateChanged = false;
		//deal with current state
		if(currentState == SheepState(EATING)) {
			this->sheepEat();
			
					
		} else if(currentState == SheepState(WALKING)) {
			this->sheepWalk();
			
		} //TODO: Running
		//if (stateChanged) {
			
		//}
		//if state has changed, do relevant things??
		ros::spinOnce();
	}
}
	
SheepNode::SheepNode() {//(int number) {
	sheepNum = 0;
	currentState = WALKING;
	
}

void SheepNode::rosSetup(int argc, char **argv) {
	std::ostringstream convert;
	ros::init(argc, argv, "sheep", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	n.getParam("sheepNum", sheepNum);
	convert << sheepNum;
	//initialise the talkies
	
	sheepMovePub = nh.advertise<se306Project::SheepMoveMsg>("sheep_" + convert.str()+ "/move", 1000);
	sheepdogPosSub = nh.subscribe<std_msgs::String pos_msg>("sheepdog_position",1000,&SheepNode::sheepDogDangerCallback,this);
	//TODO: talk to the grass, and the field?
	//TODO: talk to other sheep
	//TODO: talk to the farmer
	
	SheepNode::spin();
	
}

int main(int argc, char **argv) {

	//int number = 0;

	//ros::param::get("sheepNum", number);
	//ros::param::set("sheep/number", number+1);
	
	SheepNode sheep = SheepNode();//(number);
	
	sheep.rosSetup(argc, argv);
}

