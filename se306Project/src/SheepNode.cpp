#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "se306Project/SheepMoveMsg.h"
#include "se306Project/GrassPosMsg.h"
#include "se306Project/SheepEatMsg.h"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <ctime> // Needed to seed random number generator with a time value

enum SheepState {
	WALKING, RUNNING, EATING, WARY
};

enum SheepAgeStages {
	BIRTH, ADOLESCENCE, ADULTHOOD, OLD_AGE
};

class SheepNode {
	
public:
	//currently arbitrary constants:
	
	const static int RUNNING_LOWER_TERROR_LIMIT = 20;
	const static int WARY_LOWER_TERROR_LIMIT = 15;
	const static int GRASS_MIN_TASTY_HEIGHT = 30;
	const static int SHEEP_EAT_AMOUNT = 7;
	
	int HUNGRY_LEVEL;
	
	//setup methods
	SheepNode();
	void rosSetup(int, char**);
	void spin();	

	int sheepNum;
	SheepState currentState;
	SheepState prevState;
	SheepAgeStages age; 
	int currX;
	int currY;
	
	int terror;
	int hunger;
	int sheepAge;
	
	//movement related variables
	float prevclosestRange;
	double linear_x;
	double angular_z;
	double sheepSpeed;
	double field_X;
	double field_Y;
	
	//pose of the robot
	double px;
	double py;
	
	double theta;
	double prevpx;
	double prevpy;
	bool isRotate;
	int checkcount;
	double sdx;
	double sdy;

	void sheepEat();
	void sheepWalk();
	void sheepRun();
	
	void currentPositionCallback(se306Project::SheepMoveMsg);
	void sheepdogDangerCallback(geometry_msgs::Pose2D);
    void grassInfoCallback(se306Project::GrassPosMsg);
    bool isContainedBy(se306Project::GrassPosMsg);
	int grassNumCurrentlyBeingEaten;
	
	void SheepLifeCycle();
		
	protected:
	ros::Publisher sheepMovePub;
	ros::Publisher sheepPosePub;
	ros::Publisher grassEatPub;
	ros::Subscriber sheepdogPosSub;
    ros::Subscriber grassInfoSub;
	ros::Subscriber sheepPosSub;
};
	
SheepNode::SheepNode() {
	sheepNum = 0;
	sheepAge = 0;
	hunger = 0;
	terror = 0;
	prevState = WALKING;
	currentState = WALKING;
	age = BIRTH;
	sheepSpeed = 0.1;	
	sdx = 6;	// Setup: Based on sheepdog's initial position in world file generator.
	sdy = -0.5;
	HUNGRY_LEVEL=80;
};

void SheepNode::currentPositionCallback(se306Project::SheepMoveMsg msg) {
	if (msg.moveCommand == "UPDATE") {
		px = msg.goalX;
		py = msg.goalY;
		ROS_INFO("px: %f", px);
		ROS_INFO("py: %f", py);
	}
};

void SheepNode::sheepdogDangerCallback(geometry_msgs::Pose2D sheepdogMsg) {
		
		ROS_INFO("GOT SHEEPDOG MESSAGE %d",terror);
		if(terror < 0){
			terror = 0;
		}
		double sdxnew = sheepdogMsg.x;
		double sdynew = sheepdogMsg.y;
					
		double DistanceDiff = sqrt((abs(sdx - px)*abs(sdx - px))+(abs(sdy - py)*abs(sdy - py)));
		double DistanceDiffnew = sqrt((abs(sdxnew - px)*abs(sdxnew - px))+(abs(sdynew - py)*abs(sdynew - py))); 

		/* check the distance & find out the terror increase
		 * add the terror increase to the terror level (global var)		 * }*/
		if ((((sdxnew - sdx) != 0.0) || (sdynew - sdy != 0.0)) && DistanceDiffnew < DistanceDiff){
			if (DistanceDiffnew<=3){
				terror=terror+10;
			}else if (DistanceDiffnew<=6){
				terror=terror+5;
			}else if(DistanceDiffnew<=10){
				terror=terror+2;
			}
		//When Sheepdog is moving away from sheep
		}else if ((((sdxnew - sdx) != 0.0) || (sdynew - sdy != 0.0)) && DistanceDiffnew > DistanceDiff){
			terror = terror -5;
		}

		if (terror >= WARY_LOWER_TERROR_LIMIT && terror < RUNNING_LOWER_TERROR_LIMIT) {
			prevState = currentState;
			currentState = WARY;
		} else if (terror >= RUNNING_LOWER_TERROR_LIMIT){
			prevState = currentState;
			currentState = RUNNING;	
		}

		
		if (terror >= WARY_LOWER_TERROR_LIMIT && terror < RUNNING_LOWER_TERROR_LIMIT){
			prevState = currentState;
			currentState = WARY;
		}else if (terror >= RUNNING_LOWER_TERROR_LIMIT){
			prevState = currentState;
			currentState = RUNNING;	
		}
		
		sdx = sdxnew;
		sdy = sdynew;
	
};

void SheepNode::grassInfoCallback(se306Project::GrassPosMsg grassMsg) {
	if (SheepNode::isContainedBy(grassMsg)) {
		if (currentState == WALKING) {
			if (grassMsg.grassHeight >= GRASS_MIN_TASTY_HEIGHT || (hunger >= HUNGRY_LEVEL && grassMsg.grassHeight > 0)) {
				prevState = WALKING;
				currentState = EATING;
				grassNumCurrentlyBeingEaten = grassMsg.grassNum;
			}		
		} else if (currentState == EATING) {
			//check grass height
			if (grassMsg.grassHeight < GRASS_MIN_TASTY_HEIGHT && hunger < HUNGRY_LEVEL) {
				prevState = EATING;
				currentState = WALKING;
				grassNumCurrentlyBeingEaten = -1;
			}
		}
	} else {

	
	}
};

bool SheepNode::isContainedBy(se306Project::GrassPosMsg grassMsg) {
	//do some positioning comparisions.
	//ROS_INFO("grass%dx: %f",grassMsg.grassNum, grassMsg.x);
	//ROS_INFO("grass%dy: %f",grassMsg.grassNum, grassMsg.y);
	if (grassMsg.x - 0.45 <= px && px <= grassMsg.x + 0.45) {
		if (grassMsg.y - 0.45 <= py && py <= grassMsg.y + 0.45) {
			return true;
		}
	}
	return false;
};

void SheepNode::sheepEat(){
	ROS_INFO("EATING");
	if (prevState == SheepState(WALKING)) {
		se306Project::SheepMoveMsg msg;
		msg.moveCommand = "STOP";
		sheepMovePub.publish(msg);
	}
	prevState = EATING;
	se306Project::SheepEatMsg msg;
	msg.grassNum = grassNumCurrentlyBeingEaten;
	msg.eatAmount = SHEEP_EAT_AMOUNT;
	grassEatPub.publish(msg);
	hunger = hunger - SHEEP_EAT_AMOUNT;

};

void SheepNode::sheepWalk() {
	//publish GO to sheep_x/move (sheepMovePub)
	
	se306Project::SheepMoveMsg msg;
	msg.moveCommand = "GO";
	msg.speed = sheepSpeed;
	sheepMovePub.publish(msg);
	prevState = WALKING;
	hunger++;
	
};

void SheepNode::sheepRun() {
		
		double xDistanceDiff = (px - sdx);
		double yDistanceDiff = (py - sdy);
		ROS_INFO("SHEEPRUNCALLED-------------------------------");
		se306Project::SheepMoveMsg msg;
		msg.moveCommand = "GO";
		msg.goalX = px+xDistanceDiff;
		msg.goalY = py+yDistanceDiff;
		sheepMovePub.publish(msg);
		prevState = RUNNING;
		
		
	};

void SheepNode::SheepLifeCycle() {
	
	// Handles the different stages of a sheeps life and creates messages to be sent to sheep_move
	switch (sheepAge) {
		case 300: // 30 secs
			age = ADOLESCENCE;
			//ROS_INFO("Adolescence");
			sheepSpeed = 0.2;
			HUNGRY_LEVEL = 90;
			break;
		case 600: // 1 min
			age = ADULTHOOD;
			//ROS_INFO("Adulthood");
			sheepSpeed = 0.3;
			HUNGRY_LEVEL = 85;
			break;
		case 900: // 1 min 30 secs
			age = OLD_AGE;
			//ROS_INFO("Old age");
			HUNGRY_LEVEL = 70;
			sheepSpeed = 0.2;
			break;
	}
	sheepAge++;
};

void SheepNode::spin() {
	//do things depending on SheepState
	ros::Rate rate(10); // 1 second
	se306Project::SheepMoveMsg msg;
	msg.moveCommand = "GO";
	msg.speed = 0.1;
	sheepMovePub.publish(msg);

	while (ros::ok()) {
		//deal with state of sheep
		if (currentState == EATING) {
				if (prevState == SheepState(WALKING) || prevState == SheepState(EATING)) {
					this->sheepEat();
				} else {
					//something went wrong, set to walking?
					currentState = WALKING;
				}
		} else if (currentState == WALKING) {
				if (prevState == SheepState(RUNNING)) {
					//something went wrong, set state to WARY
					currentState = WARY;
				} else {
					this->sheepWalk();
				}
		} else if (currentState == RUNNING) {
				this->sheepRun();
		} else if (currentState == WARY) {
			if (terror >= RUNNING_LOWER_TERROR_LIMIT) {
					currentState = RUNNING;
				} else if (terror < WARY_LOWER_TERROR_LIMIT) {
					currentState = WALKING;
				}
				prevState = WARY;
		}

		this->SheepLifeCycle();
		
		// Sheep Position Publishing
		geometry_msgs::Pose2D Pose2Dmsg;
		Pose2Dmsg.x = px;
		Pose2Dmsg.y = py;
		
		sheepPosePub.publish(Pose2Dmsg);
		
		ros::spinOnce();
	
		rate.sleep();
	}
};

void SheepNode::rosSetup(int argc, char **argv) {
	std::ostringstream convert;
	ros::init(argc, argv, "sheep", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	n.getParam("sheepNum", sheepNum);
	n.getParam("fieldX", field_X);
	n.getParam("fieldY", field_Y);
	convert << sheepNum;

	//initialise the talkies
	sheepMovePub = nh.advertise<se306Project::SheepMoveMsg>("sheep_" + convert.str()+ "/move", 1000);
	sheepPosePub = nh.advertise<geometry_msgs::Pose2D>("sheep_" + convert.str()+ "/pose", 1000);
	grassEatPub = nh.advertise<se306Project::SheepEatMsg>("grass/eaten", 1000);
	sheepPosSub = nh.subscribe<se306Project::SheepMoveMsg>("sheep_" + convert.str()+ "/pos",1000, &SheepNode::currentPositionCallback,this);
	sheepdogPosSub = nh.subscribe<geometry_msgs::Pose2D>("sheepdog_position",1000, &SheepNode::sheepdogDangerCallback,this);
	grassInfoSub = nh.subscribe<se306Project::GrassPosMsg>("grass/info",1000, &SheepNode::grassInfoCallback, this);
	
	SheepNode::spin();
	
};

int main(int argc, char **argv) {
	
	SheepNode sheep = SheepNode();
	
	sheep.rosSetup(argc, argv);
};
