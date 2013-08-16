#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "se306Project/SheepMoveMsg.h"


#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
// Tunable parameters
// TODO: tune parameters as you see fit
const static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
const static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
const static float PROXIMITY_RANGE_M = 1; // Should be smaller than sensor_msgs::LaserScan::range_max
const static double FORWARD_SPEED_MPS = 0.2;
const static double ROTATE_SPEED_RADPS = M_PI/2;

class SheepMove {
			
	public:
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
	int sheepNum;
	int robotNum;
	std::string moveStatus;
	
	//methods
	SheepMove(int);
	void StageOdom_callback(nav_msgs::Odometry);
	void move(double, double);
	void statusCallback(se306Project::SheepMoveMsg);
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr&);
	void spin();
	void rosSetup(int argc, char **argv);
	
	protected:
	//talkie things
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Publisher sheepPosPub;
	ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
	ros::Subscriber sheepStatusSub; //Subscriber to sheep_[sheepNum]/move topic
	//move state things
	enum FSM fsm; // Finite state machine for the random walk algorithm
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Time rotateEndTime;
	ros::Duration rotateDuration; // Duration of the rotation
	ros::Subscriber StageOdo_sub;
};
	
void SheepMove::StageOdom_callback(nav_msgs::Odometry msg) {
	//This is the call back function to process odometry messages coming from Stage. 		
		
	px = 5 + msg.pose.pose.position.x;
	py = 10 + msg.pose.pose.position.y;
	//printf("%f",px);
		
	// If the current position is the same the previous position, then the robot is stuck and needs to move around
	if ((px == prevpx) && (py == prevpy)) {
		//msg.pose.pose.position.x = 5;
		//ROS_INFO("Prevpx: %f",prevpx);

		// Note the negative linear_x		
		//linear_x=-0.2;
		//angular_z=1;

		//theta=10;
		//px = 5;
		//py= 5;
		//printf("Robot stuck");
		if (!isRotate) {	
			ROS_INFO("Robot stuck");
			double r2 = (double)rand()/((double)RAND_MAX/(M_PI/2));
			double m2 = (double)rand()/((double)RAND_MAX/0.5);
			//ROS_INFO("r2" << r2);
			move(-m2, r2);
			
		}	
	} else {
		// One the robot becomes unstuck, then it moves around again normally
		//linear_x = 0.2;
		//angular_z = 0.2;
		//ROS_INFO("Robot unstuck");
		//checkcount=0;
	}
	//ROS_INFO("Current x position is: %f", px);
	//ROS_INFO("Current y position is: %f", py);
	prevpx = px;
	prevpy = py;
}


// Send a velocity command
void SheepMove::move(double linearVelMPS, double angularVelRadPS) {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = linearVelMPS;
	msg.angular.z = angularVelRadPS;
	commandPub.publish(msg);
}

void SheepMove::statusCallback(se306Project::SheepMoveMsg msg) {
	moveStatus = msg.moveCommand;
	//TODO: set velocity from msg
}


// Process the incoming laser scan message
void SheepMove::commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	if (fsm == FSM_MOVE_FORWARD) {
		// Compute the average range value between MIN_SCAN_ANGLE and MAX_SCAN_ANGLE
		//
		// NOTE: ideally, the following loop should have additional checks to ensure
		// that indices are not out of bounds, by computing:
		//
		//- currAngle = msg->angle_min + msg->angle_increment*currIndex
		//
		// and then ensuring that currAngle <= msg->angle_max
		unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
		unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg->angle_min) / msg->angle_increment);
		float closestRange = msg->ranges[minIndex];
		
		for (unsigned int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++) {
			if (msg->ranges[currIndex] < closestRange) {
				closestRange = msg->ranges[currIndex];
			}
		}
		//if (closestRange == prevclosestRange) {
		//	ROS_INFO_STREAM("STUCK");
		//	move(-FORWARD_SPEED_MPS, ROTATE_SPEED_RADPS);
		//	//move(0, ROTATE_SPEED_RADPS);
		//} else {
			
			//ROS_INFO_STREAM("Range: " << closestRange);
			prevclosestRange = closestRange;

			if (closestRange < PROXIMITY_RANGE_M) {
				fsm=FSM_ROTATE;
				rotateStartTime=ros::Time::now();
				//TODO: wtf is r2 for?
				float r2 = (float)rand()/((float)RAND_MAX/100);
				rotateDuration=ros::Duration(rand() % 2);
				//fsm= FSM_MOVE_FORWARD;
			}
		//}
	}
}

// Main FSM loop for ensuring that ROS messages are
// processed in a timely manner, and also for sending
// velocity controls to the simulated robot based on the FSM state
void SheepMove::spin() {
	ros::Rate rate(10); // Specify the FSM loop rate in Hz
	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
		if(moveStatus.compare("GO") == 0) {
			std_msgs::String msg;
			std::stringstream ss;
			ss << "sheep_" << sheepNum << " -- px:" << px << " py:" << py << " theta:" << theta << " isRotate:" << isRotate;
   			msg.data = ss.str();
   			
   			//ROS_INFO("%s", msg.data.c_str());
   			
			if (fsm == FSM_MOVE_FORWARD) {
				//ROS_INFO_STREAM("Start forward");
				move(FORWARD_SPEED_MPS, 0);
				checkcount++;
				if (checkcount > 3) {
					isRotate=false;
				}
			}
			if (fsm == FSM_ROTATE) {
				//ROS_INFO_STREAM("Start rotate");
				move(0, ROTATE_SPEED_RADPS);
				rotateEndTime=ros::Time::now();
				isRotate=true;
				//ROS_INFO_STREAM("Time: " << rotateEndTime);
			
				if ((rotateEndTime - rotateStartTime) > rotateDuration) {
					fsm=FSM_MOVE_FORWARD;
					//ROS_INFO_STREAM("End rotate");
					checkcount=0;
				}
			}	
		
		sheepPosPub.publish(msg);
		}//if not move, do anything?
	
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
	}
}
	
SheepMove::SheepMove(int number) {
	// Construst a new SheepMove object and hook up this ROS node
	// to the simulated robot's velocity control and laser topics
	//sheepNum = number;
	
	// Initialize random time generator
	srand(time(NULL));
	moveStatus = "GO";
	prevclosestRange = 0;
	checkcount = 0;
	prevpx = 0;
	prevpx = 0;
	sheepNum = number;
	robotNum = 0;
}	

void SheepMove::rosSetup(int argc, char **argv) {
	std::ostringstream convertS;
	std::ostringstream convertR;
	std::string out;
		
	ros::init(argc, argv, "sheepMove", ros::init_options::AnonymousName); // Initiate new ROS node named "sheepMoveX"
	ros::NodeHandle nh;
	ros::NodeHandle n("~");
	if(n.getParam("sheepNum", sheepNum))
	{
		ROS_INFO_STREAM("Got sheepNum");
	}else {
		ROS_INFO_STREAM("Dafuq, ROS?");
	}
	if(n.getParam("robotNum", robotNum))
	{

		ROS_INFO_STREAM("Got robotNum");
	}else {
		ROS_INFO_STREAM("Dafuq, ROS?");
	}
	
	
	fsm = FSM_MOVE_FORWARD;
	
	rotateStartTime = ros::Time(ros::Time::now()); 
	rotateDuration = ros::Duration(0.f);
	
	//setup talkies
	
	// Advertise a new publisher for the simulated robot's velocity command topic
	// (the second argument indicates that if multiple command messages are in
	// the queue to be sent, only the last command will be sent)
	
	convertR << robotNum; //needed as all stage robots are called robot_X
	convertS << sheepNum;
	std::string r = "robot_" + convertR.str();
	ROS_INFO_STREAM(r);
	std::string s = "sheep_" + convertS.str();
	ROS_INFO_STREAM(s);
	commandPub = nh.advertise<geometry_msgs::Twist>(r + "/cmd_vel",1000);
	sheepPosPub = nh.advertise<std_msgs::String>("sheep_position",1000);
	// Subscribe to the simulated robot's laser scan topic and tell ROS to call
	// this->commandCallback() whenever a new message is published on that topic
	laserSub = nh.subscribe<sensor_msgs::LaserScan>(r + "/base_scan", 1000, &SheepMove::commandCallback, this);
	sheepStatusSub = nh.subscribe<se306Project::SheepMoveMsg>(s + "/move", 1000, &SheepMove::statusCallback, this);
	StageOdo_sub = nh.subscribe<nav_msgs::Odometry>(r + "/odom",1000, &SheepMove::StageOdom_callback,this);
	//ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
	
	SheepMove::spin(); // Execute FSM loop
}

int main(int argc, char **argv) {
	//int number = 0;
	//ros::param::get("sheepNum", number);
	
	SheepMove walker(0); // Create new random walk object
	walker.rosSetup(argc, argv);
	return 0;
}
