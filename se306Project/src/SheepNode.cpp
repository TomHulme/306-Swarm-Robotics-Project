<<<<<<< HEAD
<<<<<<< HEAD
	#include "ros/ros.h"
	#include "std_msgs/String.h"
	#include "nav_msgs/Odometry.h"
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
		int currX;
		int currY;
		
		int terror;
		int hunger;
		
	//===movement related variables TODO: Cleanup
		float prevclosestRange;
		double linear_x;
		double angular_z;
		//pose of the robot
		double px;
		double py;
		double sdx;
		double sdy;
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
			
		protected:
		ros::Publisher sheepMovePub;
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
	
	void SheepNode::sheepdogDangerCallback(std_msgs::String sheepdogMsg) {
		std::string sheepdogPos = sheepdogMsg.data;
		int split = sheepdogPos.find(" ");
			
		double sdxnew = std::strtod(sheepdogPos.substr(0, split).c_str(),NULL);
		double sdynew = std::strtod(sheepdogPos.substr(split+1).c_str(),NULL);
			
		//Calculate the difference in distance between the sheepdog(sdx)[std_msgs::String msg?] and sheep
		//closeRange=;
		//double xDistanceDiff = abs(sdx - px);
		//double yDistanceDiff = abs(sdy - py);


		
		double DistanceDiff = sqrt((abs(sdx - px)*abs(sdx - px))+(abs(sdy - py)*abs(sdy - py)));
		double DistanceDiffnew = sqrt((abs(sdxnew - px)*abs(sdxnew - px))+(abs(sdynew - py)*abs(sdynew - py)); 

		
		
		//if (((sdxnew - sdx) != 0.0) || (sdynew - sdy != 0.0)) && DistanceDiff > DistnaceDiffnew)
		// 		{
		/* check the distance & find out the terror increase
		 * add the terror increase to the terror level (global var)		
		 * }
		
		if (terror >= WARY_LOWER_TERROR_LIMIT && terror < RUNNING_LOWER_TERROR_LIMIT)
		{
			prevState = currentState;
			currentState = WARY;
		}
		
		
		
		if (terror >= RUNNING_LOWER_TERROR_LIMIT)
		{
			prevState = currentState;
			currentState = RUNNING;	
		}
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
		if(prevState == SheepState(WALKING)) {
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
		sheepMovePub.publish(msg);
		prevState = WALKING;
		hunger++;
	}
	
	void SheepNode::sheepRun() {
		//TODO: Everything
		/* 
		 * double xDistanceDiff = abs(sdx - px);
		 * double yDistanceDiff = abs(sdy - py);
		 * 
		 * goalPositionX = px - xDistanceDiff;
		 * goalPositionY = py - yDistanceDiff;
		 *  
		 * 	se306Project::SheepMoveMsg msg;
		 * msg.moveCommand = "GO";
		 * msg.x = goalPositionX;
		 * msg.y = goalPositionY;
		 * sheepMovePub.publish(msg);
		 * prevState = RUNNING;
		 * 
		 * TODO
		 * speed
		 * 
		 * */
		//send 'run' command to SheepMove with correct direction
		//check current terror level. (may need to be a method.)
		//if terror is less than a certain level, set state WARY
		//set prevState = RUNNING
	}
	
	void SheepNode::spin() {
		//do things depending on SheepState
		ros::Rate rate(10); 
		while (ros::ok()) {
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
					//check (update?) terror level.
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
		nh = ros::NodeHandle();
		ros::NodeHandle n("~");
		n.getParam("sheepNum", sheepNum);
		convert << sheepNum;
		//initialise the talkies
		
		sheepMovePub = nh.advertise<se306Project::SheepMoveMsg>("sheep_" + convert.str()+ "/move", 1000);
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
	
=======
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "se306Project/SheepMoveMsg.h"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <ctime> // Needed to seed random number generator with a time value

enum SheepState {
	WALKING, RUNNING, EATING
};

enum SheepAgeStages {
	BIRTH, ADOLESCENCE, ADULTHOOD, OLD_AGE
};

class SheepNode {
	
public:
	//setup methods
	SheepNode();//(int);
	void rosSetup(int, char**);
	void spin();	
	void runaway(float , float );

	int sheepNum;
	SheepState currentState;
	SheepAgeStages age; 
	int currX;
	int currY;
	//parameters that need to be used eventually
	int terror;
	//int age;
	
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
	double sdx;
	double sdy;
//===

	void sheepEat();
	void sheepWalk();
	void sheepRunaway();
	
	//TODO: Sheep Danger sense
	void sheepdogDangerCallback(geometry_msgs::Pose2D sheepdogMsg);
	
	//TODO: Eating
	//void eatCallback();
	
	protected:
	ros::Publisher sheepMovePub;
	ros::Publisher sheepPosePub;
	ros::Subscriber sheepdogPosSub;
};

void SheepNode::sheepdogDangerCallback(geometry_msgs::Pose2D sheepdogMsg) {
		
	double sdx = sheepdogMsg.x;
	double sdy = sheepdogMsg.y;
		
	//Calculate the difference in distance between the sheepdog(sdx)[std_msgs::String msg?] and sheep
	//closeRange=;
	double xDistanceDiff = abs(sdx - px);
	double yDistanceDiff = abs(sdy - py);
	double distanceDiff = sqrt((xDistanceDiff)^2+(yDistanceDiff)^2);

	//if sheepdog is near: level of terror is raised depending on the distance to the sheepdog. 
	//	eg. if it is 10 units away (or whatever distance seems appropriate), then terror is low, but exists. 
	//       if it is at 6 units away, terror goes up again, and sheep move away from the dog.
	//		 if it is at 3 units away or less, sheep runs away from dog.
	
	if (distanceDiff<=3){
		terror = 70;
		if(currentState==WALKING){
			prevState = WALKING;
			currentState = RUNNING;
		}else if (currentState ==EATING) {
			prevState = EATING;
			currentState = RUNNING;
		}
	}else if (distanceDiff<=6){
		terror = 50;
		if(currentState==EATING){
			prevState = EATING;
			currentState = WALKING;
		}else if(currentState==RUNNING){
			prevState = RUNNING;
			currentState = WALKING;
		}
	}else if (distanceDiff<=10){
		terror = 10;
	}
			
										
	//ROS_INFO("Robot 0 -- Current x position is: %f", px);
	//ROS_INFO("Robot 0 -- Current y position is: %f", py);
			
	
};

void SheepNode::sheepRunaway() {
	ROS_INFO("DANGER");
};


void SheepNode::sheepEat(){
	//subscribe to current position grass service
	//use service to eat grass? (say eating, grass sends back ok to keep eating, or stop eating.
		//if stop eating, change state to walking, publish GO to sheep_x/move (sheepMovePub) 
};

void SheepNode::sheepWalk() {
	//check if there is edible grass here
		//then publish STOP to sheep_x/move (sheepMovePub) 
	//else send move message
};

void SheepNode::spin() {
	//do things depending on SheepState
	ros::Rate rate(10); // 1 second
	se306Project::SheepMoveMsg msg;
	
	msg.age= "Birth";
	sheepMovePub.publish(msg);

	int count = 0;
	while (ros::ok()) {
		//bool stateChanged = false;
		//deal with current state
		if(currentState == SheepState(EATING)) {
			this->sheepEat();
			
					
		} else if(currentState == SheepState(WALKING)) {
			this->sheepWalk();
			
		} else if (currentState == SheepState(RUNNING)){
			this->sheepRunaway();
		}


		 //TODO: Running
		//if (stateChanged) {
				//}
		//if state has changed, do relevant things??
		
		// Handles the different stages of a sheeps life and creates messages to be sent to sheep_move
		if (count == 300) { // 30 secs
			age = ADOLESCENCE;
			//ROS_INFO("Adolescence");
			msg.age = "Adolescence";
			
		} else if (count == 600) { // 1 min
			age = ADULTHOOD;
			//ROS_INFO("Adulthood");
			msg.age = "Adulthood";

		} else if (count == 900) { // 1 min 30 secs
			age = OLD_AGE;
			//ROS_INFO("Old age");
			msg.age = "OLD_AGE";
		}
		sheepMovePub.publish(msg); // Publishes the message that contains the sheeps life stage
		count++;

	
		// Sheep Position Publishing
		geometry_msgs::Pose2D Pose2Dmsg;
		Pose2Dmsg.x = px;
		Pose2Dmsg.y = py;
		
		sheepPosePub.publish(Pose2Dmsg);
		
		
		ros::spinOnce();
	
		//ROS_INFO("Count: %d", count);
		rate.sleep();
	}
};
	
SheepNode::SheepNode() {//(int number) {
	sheepNum = 0;
	currentState = WALKING;
	age = BIRTH;
	
};

void SheepNode::rosSetup(int argc, char **argv) {
	std::ostringstream convert;
	ros::init(argc, argv, "sheep", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	n.getParam("sheepNum", sheepNum);
	convert << sheepNum;
	//initialise the talkies
	
	sheepMovePub = nh.advertise<se306Project::SheepMoveMsg>("sheep_" + convert.str()+ "/move", 1000);
	sheepPosePub = nh.advertise<geometry_msgs::Pose2D>("sheep_" + convert.str()+ "/pose", 1000);
	sheepdogPosSub = nh.subscribe<geometry_msgs::Pose2D>("sheepdog_position",1000, &SheepNode::sheepdogDangerCallback,this);

	//TODO: talk to the grass, and the field?
	//TODO: talk to other sheep
	//TODO: talk to the farmer
	
	SheepNode::spin();
};

int main(int argc, char **argv) {

	//int number = 0;

	//ros::param::get("sheepNum", number);
	//ros::param::set("sheep/number", number+1);
	
	SheepNode sheep = SheepNode();//(number);
	
	sheep.rosSetup(argc, argv);
};

>>>>>>> efc54828ac908ee71ef5333825729a7b266e075a
=======
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "se306Project/SheepMoveMsg.h"

#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <ctime> // Needed to seed random number generator with a time value

enum SheepState {
	WALKING, RUNNING, EATING
};

enum SheepAgeStages {
	BIRTH, ADOLESCENCE, ADULTHOOD, OLD_AGE
};

class SheepNode {
	
public:
	//setup methods
	SheepNode();//(int);
	void rosSetup(int, char**);
	void spin();	
	void runaway(float , float );

	int sheepNum;
	SheepState currentState;
	SheepAgeStages age; 
	int currX;
	int currY;
	//parameters that need to be used eventually
	int terror;
	//int age;
	
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
	double sdx;
	double sdy;
//===

	void sheepEat();
	void sheepWalk();
	void sheepRunaway();
	
	//TODO: Sheep Danger sense
	void sheepdogDangerCallback(geometry_msgs::Pose2D sheepdogMsg);
	
	//TODO: Eating
	//void eatCallback();
	
	protected:
	ros::Publisher sheepMovePub;
	ros::Publisher sheepPosePub;
	ros::Subscriber sheepdogPosSub;
};

void SheepNode::sheepdogDangerCallback(geometry_msgs::Pose2D sheepdogMsg) {
		
	double sdx = sheepdogMsg.x;
	double sdy = sheepdogMsg.y;
		
	//Calculate the difference in distance between the sheepdog(sdx)[std_msgs::String msg?] and sheep
	//closeRange=;
	double xDistanceDiff = abs(sdx - px);
	double yDistanceDiff = abs(sdy - py);
	double distanceDiff = sqrt((xDistanceDiff)^2+(yDistanceDiff)^2);

	//if sheepdog is near: level of terror is raised depending on the distance to the sheepdog. 
	//	eg. if it is 10 units away (or whatever distance seems appropriate), then terror is low, but exists. 
	//       if it is at 6 units away, terror goes up again, and sheep move away from the dog.
	//		 if it is at 3 units away or less, sheep runs away from dog.
	
	if (distanceDiff<=3){
		terror = 70;
		if(currentState==WALKING){
			prevState = WALKING;
			currentState = RUNNING;
		}else if (currentState ==EATING) {
			prevState = EATING;
			currentState = RUNNING;
		}
	}else if (distanceDiff<=6){
		terror = 50;
		if(currentState==EATING){
			prevState = EATING;
			currentState = WALKING;
		}else if(currentState==RUNNING){
			prevState = RUNNING;
			currentState = WALKING;
		}
	}else if (distanceDiff<=10){
		terror = 10;
	}
			
										
	//ROS_INFO("Robot 0 -- Current x position is: %f", px);
	//ROS_INFO("Robot 0 -- Current y position is: %f", py);
			
	
};

void SheepNode::sheepRunaway() {
	ROS_INFO("DANGER");
};


void SheepNode::sheepEat(){
	//subscribe to current position grass service
	//use service to eat grass? (say eating, grass sends back ok to keep eating, or stop eating.
		//if stop eating, change state to walking, publish GO to sheep_x/move (sheepMovePub) 
};

void SheepNode::sheepWalk() {
	//check if there is edible grass here
		//then publish STOP to sheep_x/move (sheepMovePub) 
	//else send move message
};

void SheepNode::spin() {
	//do things depending on SheepState
	ros::Rate rate(10); // 1 second
	se306Project::SheepMoveMsg msg;
	
	msg.age= "Birth";
	sheepMovePub.publish(msg);

	int count = 0;
	while (ros::ok()) {
		//bool stateChanged = false;
		//deal with current state
		if(currentState == SheepState(EATING)) {
			this->sheepEat();
			
					
		} else if(currentState == SheepState(WALKING)) {
			this->sheepWalk();
			
		} else if (currentState == SheepState(RUNNING)){
			this->sheepRunaway();
		}


		 //TODO: Running
		//if (stateChanged) {
				//}
		//if state has changed, do relevant things??
		
		// Handles the different stages of a sheeps life and creates messages to be sent to sheep_move
		if (count == 300) { // 30 secs
			age = ADOLESCENCE;
			//ROS_INFO("Adolescence");
			msg.age = "Adolescence";
			
		} else if (count == 600) { // 1 min
			age = ADULTHOOD;
			//ROS_INFO("Adulthood");
			msg.age = "Adulthood";

		} else if (count == 900) { // 1 min 30 secs
			age = OLD_AGE;
			//ROS_INFO("Old age");
			msg.age = "OLD_AGE";
		}
		sheepMovePub.publish(msg); // Publishes the message that contains the sheeps life stage
		count++;

	
		// Sheep Position Publishing
		geometry_msgs::Pose2D Pose2Dmsg;
		Pose2Dmsg.x = px;
		Pose2Dmsg.y = py;
		
		sheepPosePub.publish(Pose2Dmsg);
		
		
		ros::spinOnce();
	
		//ROS_INFO("Count: %d", count);
		rate.sleep();
	}
};
	
SheepNode::SheepNode() {//(int number) {
	sheepNum = 0;
	currentState = WALKING;
	age = BIRTH;
	
};

void SheepNode::rosSetup(int argc, char **argv) {
	std::ostringstream convert;
	ros::init(argc, argv, "sheep", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	n.getParam("sheepNum", sheepNum);
	convert << sheepNum;
	//initialise the talkies
	
	sheepMovePub = nh.advertise<se306Project::SheepMoveMsg>("sheep_" + convert.str()+ "/move", 1000);
	sheepPosePub = nh.advertise<geometry_msgs::Pose2D>("sheep_" + convert.str()+ "/pose", 1000);
	sheepdogPosSub = nh.subscribe<geometry_msgs::Pose2D>("sheepdog_position",1000, &SheepNode::sheepdogDangerCallback,this);

	//TODO: talk to the grass, and the field?
	//TODO: talk to other sheep
	//TODO: talk to the farmer
	
	SheepNode::spin();
};

int main(int argc, char **argv) {

	//int number = 0;

	//ros::param::get("sheepNum", number);
	//ros::param::set("sheep/number", number+1);
	
	SheepNode sheep = SheepNode();//(number);
	
	sheep.rosSetup(argc, argv);
};

>>>>>>> efc54828ac908ee71ef5333825729a7b266e075a
