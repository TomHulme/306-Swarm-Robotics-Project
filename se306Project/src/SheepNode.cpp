#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "se306Project/SheepMoveMsg.h"
#include "se306Project/GrassInfoMsg.h"
#include "se306Project/SheepEatMsg.h"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <ctime> // Needed to seed random number generator with a time value

enum SheepState {
	WALKING, RUNNING, EATING, WARY
};

enum SheepAgeStages {
	BIRTH, ADOLESCENCE, ADULTHOOD, OLD_AGE
};

//currently arbitrary constants:
const static int HUNGRY_LEVEL = 80;
const static int RUNNING_LOWER_TERROR_LIMIT = 70;
const static int WARY_LOWER_TERROR_LIMIT = 50;
const static int GRASS_MIN_TASTY_HEIGHT = 30;
const static int SHEEP_EAT_AMOUNT = 7;

class SheepNode {
	
public:
	//setup methods
	SheepNode();
	void rosSetup(int, char**);
	void spin();	
	
	ros::NodeHandle nh;
	int sheepNum;
	SheepState currentState;
	SheepState prevState;
	SheepAgeStages age; 
	int currX;
	int currY;
	
	int terror;
	int hunger;
	int sheepAge;
	
//===movement related variables TODO: Cleanup
	float prevclosestRange;
	double linear_x;
	double angular_z;
	double sheepSpeed;
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
	void sheepRun();
	
	//TODO: Sheep Danger sense
	void sheepdogDangerCallback(std_msgs::String);
	//TODO: Eating
    void grassInfoCallback(se306Project::GrassInfoMsg);
    bool isContainedBy(se306Project::GrassInfoMsg);
	
	void SheepLifeCycle();
		
	protected:
	ros::Publisher sheepMovePub;
	ros::Publisher sheepPosePub;
	ros::Publisher grassEatPub;
	ros::Subscriber sheepdogPosSub;
    ros::Subscriber grassInfoSub;
};

void SheepNode::grassInfoCallback(se306Project::GrassInfoMsg grassMsg) {
	if (SheepNode::isContainedBy(grassMsg)) {
		if (currentState == WALKING) {
			if (grassMsg.grassHeight >= GRASS_MIN_TASTY_HEIGHT || hunger > HUNGRY_LEVEL) {
				prevState = WALKING;
				currentState = EATING;
				std::ostringstream grassNum;
				grassNum << grassMsg.grassNum;
				grassEatPub = nh.advertise<se306Project::SheepEatMsg>("grass" + grassNum.str() + "/eaten", 1000);
			}		
		} else if (currentState == EATING) {
			//check grass height
			if (grassMsg.grassHeight < GRASS_MIN_TASTY_HEIGHT) {
				prevState = EATING;
				currentState = WALKING;
			}
		}
	} else {
		//	TODO:	compare msg to current position. 
		//	TODO: 	if grass position is closer than current goal then:
		//	TODO:		set new goal
		
	}
}

bool SheepNode::isContainedBy(se306Project::GrassInfoMsg grassMsg) {
	//do some positioning comparisions.
	//if (x <= px && px <= x + 90(?)) {
	//	if (y <= py && py <= y + 90(?)) {
	//		return true;
	//	}
	//}
	return false;
}

void SheepNode::sheepdogDangerCallback(geometry_msgs::Pose2D sheepdogMsg) {
		
	double sdx = sheepdogMsg.x;
	double sdy = sheepdogMsg.y;
		
	//Calculate the difference in distance between the sheepdog(sdx)[std_msgs::String msg?] and sheep
	//closeRange=;
	double xDistanceDiff = abs(sdx - px);
	double yDistanceDiff = abs(sdy - py);

	//if sheepdog is near: level of terror is raised depending on the distance to the sheepdog. 
	//	eg. if it is 10 units away (or whatever distance seems appropriate), then terror is low, but exists. 
	//       if it is at 6 units away, terror goes up again, and sheep move away from the dog.
	//		 if it is at 3 units away or less, sheep runs away from dog.
	if (xDistanceDiff<=5||yDistanceDiff<=5){
		runaway(sdx,sdy);
	}									
	ROS_INFO("Robot 0 -- Current x position is: %f", px);
	ROS_INFO("Robot 0 -- Current y position is: %f", py);
			
	
};

void runaway(float sdx, float sdy) {
	ROS_INFO("x Distance between Sheepdog and Sheep: %f",sdx);
	ROS_INFO("y Distance between Sheepdog and Sheep: %f",sdy);
};


void SheepNode::sheepEat(){
	if (prevState == SheepState(WALKING)) {
		se306Project::SheepMoveMsg msg;
		msg.moveCommand = "STOP";
		sheepMovePub.publish(msg);
	}
	prevState = EATING;
	se306Project::SheepEatMsg msg;
	msg.eatAmount = SHEEP_EAT_AMOUNT;
	grassEatPub.publish(msg);
	hunger = hunger - SHEEP_EAT_AMOUNT;
}

void SheepNode::sheepWalk() {
	 
	//publish GO to sheep_x/move (sheepMovePub)
	se306Project::SheepMoveMsg msg;
	msg.moveCommand = "GO";
	//TODO: msg directions
	msg.speed = sheepSpeed;
	sheepMovePub.publish(msg);
	prevState = WALKING;
	hunger++;
}

void SheepNode::sheepRun() {
	//TODO: Everything
	//check current sheepdog position
	//send 'run' command to SheepMove with correct direction and double the current speed
	//check current terror level. (may need to be a method.)
	//if terror is less than a certain level, set state WARY
	//set prevState = RUNNING
}

void SheepNode::SheepLifeCycle() {
// Handles the different stages of a sheeps life and creates messages to be sent to sheep_move

	switch (sheepAge) {
		case 300: // 30 secs
			age = ADOLESCENCE;
			//ROS_INFO("Adolescence");
			sheepSpeed = 0.2;
			break;
		case 600: // 1 min
			age = ADULTHOOD;
			//ROS_INFO("Adulthood");
			sheepSpeed = 0.3;
			break;
		case 900: // 1 min 30 secs
			age = OLD_AGE;
			//ROS_INFO("Old age");
			sheepSpeed = 0.2;
			break;
			
	}
	sheepAge++;
}

void SheepNode::spin() {
	//do things depending on SheepState
	ros::Rate rate(10); 
	while (ros::ok()) {
		ROS_INFO_STREAM(currentState);
		//deal with state of sheep
		switch (currentState) {
			case EATING:
				if (prevState == SheepState(WALKING) || prevState == SheepState(EATING)) {
					this->sheepEat();
				} else {
					//something went wrong, set to walking?
					currentState = WALKING;
				}
				break;
			case WALKING:
				if (prevState == SheepState(RUNNING)) {
					//something went wrong, set state to WARY
					currentState = WARY;
				} else {
					this->sheepWalk();
				}
				break;
			case RUNNING:
				this->sheepRun();
				break;
			case WARY:
				if (terror >= RUNNING_LOWER_TERROR_LIMIT) {
					currentState = RUNNING;
				} else if (terror < WARY_LOWER_TERROR_LIMIT) {
					currentState = WALKING;
				}
				prevState = WARY;
				break;
			//TODO: case FOLLOWING:
			//	if prevState != RUNNING
			//		do follow logic
			//		set prevState = FOLLOWING
			
		}
		//deal with age (and illness?)
		this->SheepLifeCycle();
		
		ros::spinOnce();
	}
}

SheepNode::SheepNode() {
	sheepNum = 0;
	sheepAge = 0;
	hunger = 0;
	terror = 0;
	currentState = WALKING;
	age = BIRTH;
	sheepSpeed = 0.1;	
}

void SheepNode::rosSetup(int argc, char **argv) {
	std::ostringstream convert;
	ros::init(argc, argv, "sheep", ros::init_options::AnonymousName);
	nh = ros::NodeHandle();
	ros::NodeHandle n("~");
	n.getParam("sheepNum", sheepNum);
	convert << sheepNum;
	//initialise the talkies
	
	sheepMovePub = nh.advertise<se306Project::SheepMoveMsg>("sheep_" + convert.str()+ "/move", 1000);
	sheepPosePub = nh.advertise<geometry_msgs::Pose2D>("sheep_" + convert.str()+ "/pose", 1000);
	sheepdogPosSub = nh.subscribe<std_msgs::String>("sheepdog_position",1000, &SheepNode::sheepdogDangerCallback,this);
	grassInfoSub = nh.subscribe<se306Project::GrassInfoMsg>("grass/info",1000, &SheepNode::grassInfoCallback, this);
	//TODO: talk to the field?
	//TODO: talk to other sheep
	//TODO: talk to the farmer
	
	SheepNode::spin();
	
}

int main(int argc, char **argv) {

	SheepNode sheep = SheepNode();
	
	sheep.rosSetup(argc, argv);
}

