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
	WALKING, RUNNING, EATING, WARY
};

class SheepNode {
	
public:
	//setup methods
	SheepNode();//(int);
	void rosSetup(int, char**);
	void spin();	
	
	ros::NodeHandle nh;
	int sheepNum;
	SheepState currentState;
	SheepState prevState;
	int currX;
	int currY;
	//parameters that need to be used eventually
	int terror;
	int hunger;
	
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
	void sheepdogDangerCallback(std_msgs::String);
	//TODO: Eating
    void grassInfoCallback(se306Project::GrassInfoMsg);
	//void eatCallback();
	
	protected:
	ros::Publisher sheepMovePub;
	ros::Publisher grassEatPub;
	ros::Subscriber sheepdogPosSub;
    ros::Subscriber grassInfoSub;
};

void SheepNode::grassInfoCallback(se306Project::GrassInfoMsg grassMsg) {
	if (currentState == WALKING) {
		//	TODO:	compare msg to current position. 
		//	TODO: 	if grass position is closer than current goal then:
		//	TODO:		set new goal
		//	if sheep is in same position as grass (grass is under the sheep) then:
		if (SheepNode::isContainedBy(grassMsg)) {
			if (grassMsg.height > 30 || hunger > 80) {
				prevState = SheepState(WALKING);
				currentState = SheepState(EATING);
				grassEatPub = nh.advertise<se306Project::SheepEatMsg>("grass" + convert.str() + "/eaten", 1000);
			}		
		}
	}
}

void SheepNode::sheepdogDangerCallback(std_msgs::String sheepdogMsg) {
	std::string sheepdogPos = sheepdogMsg.data;
	int split = sheepdogPos.find(" ");
		
	double sdx = std::strtod(sheepdogPos.substr(0, split).c_str(),NULL);
	double sdy = std::strtod(sheepdogPos.substr(split+1).c_str(),NULL);
		
	//Calculate the difference in distance between the sheepdog(sdx)[std_msgs::String msg?] and sheep
	//closeRange=;
	double xDistanceDiff = abs(sdx - px);
	double yDistanceDiff = abs(sdy - py);

	//if sheepdog is near: level of terror is raised depending on the distance to the sheepdog. 
	//	eg. if it is 10 units away (or whatever distance seems appropriate), then terror is low, but exists. 
	//       if it is at 6 units away, terror goes up again, and sheep move away from the dog.
	//		 if it is at 3 units away or less, sheep runs away from dog.
	//if (xDistanceDiff<=20||yDistanceDiff<=20){
	//	scared(sdx,sdy);
	//}
	//TODO: Set CurrentState: RUNNING. 
}


void SheepNode::sheepEat(){
	//send eating on grassEatPub
	//decrease hunger
}

void SheepNode::sheepWalk() {
	//if prevState != WALKING
	//	TODO: publish GO and direction to sheep_x/move (sheepMovePub) [goal movement]
	//else
	//	publish GO to sheep_x/move (sheepMovePub)
	//
	hunger++;
}

void SheepNode::spin() {
	//do things depending on SheepState
	ros::Rate rate(10); 
	while (ros::ok()) {
		switch (currentState) {
			case EATING:
				if (prevState != SheepState(RUNNING)) {
					if(prevState == SheepState(WALKING)) {
						//publish STOP to sheep_x/move (sheepMovePub)
					}
					this->sheepEat();
				}
				break;
			case WALKING:
				this->sheepWalk();
				break;
			case RUNNING:
				//check current sheepdog position
				//send 'run' command to SheepMove with correct direction
				//check current terror level. (may need to be a method.)
				//if terror is less than a certain level, set state WARY
				break;
			case WARY:
				//check terror level.
				//if higher than a certain level, set state RUNNING
				//
			
		}
		//deal with 
		ros::spinOnce();
	}
}

SheepNode::SheepNode() {
	sheepNum = 0;
	hunger = 0;
	terror = 0;
	currentState = WALKING;
	
}

void SheepNode::rosSetup(int argc, char **argv) {
	std::ostringstream convert;
	ros::init(argc, argv, "sheep", ros::init_options::AnonymousName);
	nh = NodeHandle();
	ros::NodeHandle n("~");
	n.getParam("sheepNum", sheepNum);
	convert << sheepNum;
	//initialise the talkies
	
	sheepMovePub = nh.advertise<se306Project::SheepMoveMsg>("sheep_" + convert.str()+ "/move", 1000);
	sheepdogPosSub = nh.subscribe<std_msgs::String>("sheepdog_position",1000, &SheepNode::sheepdogDangerCallback,this);
	grassInfoSub = nh.subscribe<se306Project::GrassInfoMsg>("grass/info",1000, &SheepNode::grassInfoCallback, this);
	//TODO: talk to the grass
	//TODO: talk to the field?
	//TODO: talk to other sheep
	//TODO: talk to the farmer
	
	SheepNode::spin();
	
}

int main(int argc, char **argv) {

	SheepNode sheep = SheepNode();
	
	sheep.rosSetup(argc, argv);
}
