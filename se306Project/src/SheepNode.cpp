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
	const static int HUNGRY_LEVEL = 80;
	const static int RUNNING_LOWER_TERROR_LIMIT = 70;
	const static int WARY_LOWER_TERROR_LIMIT = 50;
	const static int GRASS_MIN_TASTY_HEIGHT = 30;
	const static int SHEEP_EAT_AMOUNT = 7;
	//sheepdog terror distance
	const static int TERROR_LEVEL_ONE = 5;
	const static int TERROR_LEVEL_TWO = 4;
	const static int TERROR_LEVEL_THREE = 2.5;
	const static int TERROR_LEVEL_FOUR = 1;
	
	
	//setup methods
	SheepNode();
	void rosSetup(int, char**);
	void spin();	
	
	//ros::NodeHandle nh;
	int sheepNum;
	SheepState currentState;
	SheepState prevState;
	SheepAgeStages age; 
	bool goalSet;
	double goalX;
	double goalY;
	
	int terror;
	int hunger;
	int sheepAge;
	
//===Position related variables
	double sheepSpeed;
	double field_X;
	double field_Y;
	
	double px; //xPos of the sheep
	double py; //yPos of the sheep
	double sdx; //xPos of the sheepdog
	double sdy; //yPos of the sheepdog
//===
	//State Methods
	void sheepEat();
	void sheepWalk();
	void sheepRun();
	void sheepWary();
	
	void currentPositionCallback(se306Project::SheepMoveMsg);
	//TODO: Sheep Danger sense
	void sheepdogDangerCallback(geometry_msgs::Pose2D);
	//TODO: Eating
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
void SheepNode::currentPositionCallback(se306Project::SheepMoveMsg msg) {
	if (msg.moveCommand == "UPDATE") {
		px = msg.goalX;
		py = msg.goalY;
		ROS_INFO("px: %f", px);
		ROS_INFO("py: %f", py);
	}
};

void SheepNode::sheepdogDangerCallback(geometry_msgs::Pose2D sheepdogMsg) {
	double sdxnew = sheepdogMsg.x;
	double sdynew = sheepdogMsg.y;
		
	double DistanceDiff = sqrt((abs(sdx - px)*abs(sdx - px))+(abs(sdy - py)*abs(sdy - py)));
	double DistanceDiffNew = sqrt((abs(sdxnew - px)*abs(sdxnew - px))+(abs(sdynew - py)*abs(sdynew - py))); 
	if (DistanceDiff > DistanceDiffNew) {
		/* check the distance & find out the terror increase
		 * add the terror increase to the terror level (global var)	
		*/
		if (DistanceDiffNew <= TERROR_LEVEL_ONE && DistanceDiffNew > TERROR_LEVEL_TWO) {
			terror = terror + 2;
		} else if (DistanceDiffNew <= TERROR_LEVEL_TWO && DistanceDiffNew > TERROR_LEVEL_THREE) {
			terror = terror + 5;
		} else if (DistanceDiffNew <= TERROR_LEVEL_THREE && DistanceDiffNew > TERROR_LEVEL_FOUR) {
			terror = terror + 15;
		} else if (DistanceDiffNew < TERROR_LEVEL_FOUR) {
			terror = terror + 35;
		}
		if (terror > 100)
			terror = 100;
		ROS_INFO("Sheepdog is closer. Terror at %d", terror);
	} else {
		terror = terror - 4;
		if (terror < 0)
			terror = 0;
		ROS_INFO("Sheepdog is not closer. Terror at %d", terror);
	}
	
	if (terror >= WARY_LOWER_TERROR_LIMIT && terror < RUNNING_LOWER_TERROR_LIMIT){
		prevState = currentState;
		currentState = WARY;
	}
	if (terror >= RUNNING_LOWER_TERROR_LIMIT) {
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
		if(goalSet) {
			double DistanceDiff = sqrt((abs(goalX - px)*abs(goalX - px))+(abs(goalY - py)*abs(goalY - py)));
			double DistanceDiffNew = sqrt((abs(grassMsg.x - px)*abs(grassMsg.x - px))+(abs(grassMsg.y - py)*abs(grassMsg.y - py))); 
			if (DistanceDiff > DistanceDiffNew && (grassMsg.grassHeight >= GRASS_MIN_TASTY_HEIGHT || (hunger >= HUNGRY_LEVEL && grassMsg.grassHeight > 0))) {
				//new grass is closer
				ROS_INFO("Setting new goal: %d (%f,%f)", grassMsg.grassNum, grassMsg.x, grassMsg.y);
				goalSet = true;
				goalX = grassMsg.x;
				goalY = grassMsg.y;
			}
		} else {
			if (grassMsg.grassHeight >= GRASS_MIN_TASTY_HEIGHT || (hunger >= HUNGRY_LEVEL && grassMsg.grassHeight > 0)) {
				ROS_INFO("Setting new goal: %d (%f,%f)", grassMsg.grassNum, grassMsg.x, grassMsg.y);
				goalSet = true;
				goalX = grassMsg.x;
				goalY = grassMsg.y;
			}
		}
	}
};

bool SheepNode::isContainedBy(se306Project::GrassPosMsg grassMsg) {
	//do some positioning comparisions.
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
		goalSet = false;
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
	if(goalSet) { //allow SheepMove to do it's thing if there is no goal set
		msg.goalX = goalX;
		msg.goalY = goalY;
	}
	msg.speed = sheepSpeed;
	sheepMovePub.publish(msg);
	prevState = WALKING;
	hunger++;
	
};

void SheepNode::sheepRun() {
	ROS_INFO("Sheep is terrorfied and running");
	//TODO:
	///This code is untested, and probably won't be implemented in time for final, but this is a way 
	///of doing basic evasion of the sheepdog by running directly away from it, with a small amount of 
	///variance to cause a curve one way or another
	 
	double xDistanceDiff = abs(sdx - px);
	double yDistanceDiff = abs(sdy - py);
	//introduces a small amount of directional variation, should cause the path to tend one way or the other
	double xRand = (rand() % 2)/10.0 - 0.1; 
	double yRand = (rand() % 2)/10.0 - 0.1;
	goalX = px - xDistanceDiff + xRand;
	goalY = py - yDistanceDiff + yRand;
	//need to alter goalX + goalY to keep goal in field
	goalSet = true;
	
	se306Project::SheepMoveMsg msg;
	msg.moveCommand = "GO";
	msg.goalX = goalX;
	msg.goalY = goalY;
	msg.speed = sheepSpeed*2;
	sheepMovePub.publish(msg);
	prevState = RUNNING;
};

void SheepNode::sheepWary() {
	if (prevState != WARY) {
		se306Project::SheepMoveMsg msg;
		msg.moveCommand = "STOP";
		sheepMovePub.publish(msg);
		goalSet = false;
	}
	if (terror >= RUNNING_LOWER_TERROR_LIMIT) {
		currentState = RUNNING;
	} else if (terror < WARY_LOWER_TERROR_LIMIT) {
		currentState = WALKING;
	}
	prevState = WARY;
};
			
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
};

void SheepNode::spin() {
	//do things depending on SheepState
	ros::Rate rate(10); // 1 second

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
			this->sheepWary();
		}
			//TODO: case FOLLOWING:
			//	if prevState != RUNNING
			//		do follow logic
			//		set prevState = FOLLOWING
			
		this->SheepLifeCycle();

		//ROS_INFO("fieldx: %d", field_X);
		//ROS_INFO("fieldy: %d", field_Y);
		
		// Sheep Position Publishing
		geometry_msgs::Pose2D Pose2Dmsg;
		Pose2Dmsg.x = px;
		Pose2Dmsg.y = py;
		
		sheepPosePub.publish(Pose2Dmsg);
		
		ros::spinOnce();
		//rate.sleep();
	}
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
	goalSet = false;
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
	
	//TODO: talk to the grass, and the field?
	//TODO: talk to other sheep
	//TODO: talk to the farmer
	
	SheepNode::spin();
	
};

int main(int argc, char **argv) {
	
	SheepNode sheep = SheepNode();
	
	sheep.rosSetup(argc, argv);
};