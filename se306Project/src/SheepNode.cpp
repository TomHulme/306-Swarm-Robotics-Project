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

enum SheepAgeStages {
	BIRTH, ADOLESCENCE, ADULTHOOD, OLD_AGE
};

class SheepNode {
	
public:
	//setup methods
	SheepNode();//(int);
	void rosSetup(int, char**);
	void spin();	
	
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
//===

	void sheepEat();
	void sheepWalk();
	
	//TODO: Sheep Danger sense
	void sheepdogDangerCallback(std_msgs::String);
	//TODO: Eating
	//void eatCallback();
	
	protected:
	ros::Publisher sheepMovePub;
	ros::Subscriber sheepdogPosSub;
};

void SheepNode::sheepdogDangerCallback(std_msgs::String sheepdogMsg) {
	std::string sheepdogPos = sheepdogMsg.data;//.c_str();
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
	int count = 0;
	while (ros::ok()) {
		//bool stateChanged = false;
		//deal with current state
		if(currentState == SheepState(EATING)) {
			this->sheepEat();
			
					
		} else if(currentState == SheepState(WALKING)) {
			this->sheepWalk();
			
		} //TODO: Running
		//if (stateChanged) {
		
		if (count == 300) { // 30 secs
			age = ADOLESCENCE;
			ROS_INFO("Adolescence");
		} else if (count == 600) { // 1 min
			age = ADULTHOOD;
			ROS_INFO("Adulthood");
		} else if (count == 900) { // 1 min 30 secs
			age = OLD_AGE;
			ROS_INFO("Old age");
		}
		//}
		//if state has changed, do relevant things??
		ros::spinOnce();
		count++;
		//ROS_INFO("Count: %d", count);
		rate.sleep();
	}
}
	
SheepNode::SheepNode() {//(int number) {
	sheepNum = 0;
	currentState = WALKING;
	age = BIRTH;
	
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
	sheepdogPosSub = nh.subscribe<std_msgs::String>("sheepdog_position",1000, &SheepNode::sheepdogDangerCallback,this);

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

