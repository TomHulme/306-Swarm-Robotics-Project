#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"


#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

float prevclosestRange = 0;
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;
double prevpx;
double prevpy;
bool isRotate;

int checkcount=0;

class RandomWalk {
		public:
		// Construst a new RandomWalk object and hook up this ROS node
		// to the simulated robot's velocity control and laser topics
		RandomWalk(ros::NodeHandle& nh) :
		fsm(FSM_MOVE_FORWARD),
		rotateStartTime(ros::Time::now()),
		rotateDuration(0.f) {
		// Initialize random time generator
		srand(time(NULL));
		// Advertise a new publisher for the simulated robot's velocity command topic
		// (the second argument indicates that if multiple command messages are in
		// the queue to be sent, only the last command will be sent)
		ros::NodeHandle n;
		commandPub = nh.advertise<geometry_msgs::Twist>("robot_4/cmd_vel",1000);
		sheepPosPub = nh.advertise<std_msgs::String>("sheep_position",1000);
		// Subscribe to the simulated robot's laser scan topic and tell ROS to call
		// this->commandCallback() whenever a new message is published on that topic
		laserSub = nh.subscribe<sensor_msgs::LaserScan>("robot_4/base_scan", 1000, &RandomWalk::commandCallback, this);
		StageOdo_sub = nh.subscribe<nav_msgs::Odometry>("robot_4/odom",1000, &RandomWalk::StageOdom_callback,this);
		//ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_4/odom",1000, StageOdom_callback);

	};

	void StageOdom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 		
		
		px = 5 + msg.pose.pose.position.x;
		py =10 + msg.pose.pose.position.y;
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
		ROS_INFO("Robot 4 -- Current x position is: %f", px);
		ROS_INFO("Robot 4 -- Current y position is: %f", py);
		prevpx = px;
		prevpy = py;
	};


	// Send a velocity command
	void move(double linearVelMPS, double angularVelRadPS) {
		geometry_msgs::Twist msg; // The default constructor will set all commands to 0
		msg.linear.x = linearVelMPS;
		msg.angular.z = angularVelRadPS;
		commandPub.publish(msg);
	};

	


	// Process the incoming laser scan message
	void commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
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
					float r2 = (float)rand()/((float)RAND_MAX/100);
					rotateDuration=ros::Duration(rand() % 2);
					//fsm= FSM_MOVE_FORWARD;
				}
			//}

		}
	};
	// Main FSM loop for ensuring that ROS messages are
	// processed in a timely manner, and also for sending
	// velocity controls to the simulated robot based on the FSM state
	void spin() {
		ros::Rate rate(5); // Specify the FSM loop rate in Hz
		while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
			std_msgs::String msg;
			std::stringstream ss;
			ss << "robot_4 -- px:" << px << " py:" << py << " theta:" << theta << " isRotate:" << isRotate;
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

		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
		}
	};
	enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
	// Tunable parameters
	// TODO: tune parameters as you see fit
	const static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
	const static float PROXIMITY_RANGE_M = 1; // Should be smaller than sensor_msgs::LaserScan::range_max
	const static double FORWARD_SPEED_MPS = 0.2;
	const static double ROTATE_SPEED_RADPS = M_PI/2;
	
	protected:
	ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
	ros::Publisher sheepPosPub;
	ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
	enum FSM fsm; // Finite state machine for the random walk algorithm
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Time rotateEndTime;
	ros::Duration rotateDuration; // Duration of the rotation
	ros::Subscriber StageOdo_sub;
	

};
int main(int argc, char **argv) {
	ros::init(argc, argv, "RobotNode4"); // Initiate new ROS node named "random_walk"
	ros::NodeHandle n;
	prevpx = 0;
	prevpx= 0;
	RandomWalk walker(n); // Create new random walk object
	walker.spin(); // Execute FSM loop
	return 0;
};

