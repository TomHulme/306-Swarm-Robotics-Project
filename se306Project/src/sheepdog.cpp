//============================================================================
// Name        : sheepdog.cpp
// Author      : Tom Hulme
// Date		   : 28/08/2013
// Version     : 
// Description : Contains all the sheepdog methods
//============================================================================
#include <se306Project/sheepdog.h>

// static variables
const static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
const static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
const static float PROXIMITY_RANGE_M = 5;
const static double FORWARD_SPEED_MPS = 1;
const static double ROTATE_SPEED_RADPS = M_PI;

// ranger and movement variables
float prevclosestRange = 0;
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double tz;
double theta;
double prevpx;
double prevpy;
bool isRotate;

// truck position
double truckX = -1.0;
double truckY = -1.0;

// counters
//sheepNum = 0;
int checkcount = 0;

// Finite State Machines
enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
enum FSM fsm; // Finite state machine for the random walk algorithm

// ROS Publishers and Subscribers
ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
ros::Publisher sheepdogPosPub;
ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
ros::Subscriber truckPosSub;
ros::Time rotateStartTime; // Start time of the rotation
ros::Time rotateEndTime;
ros::Duration rotateDuration; // Duration of the rotation
ros::Subscriber StageOdo_sub;
laser_geometry::LaserProjection projector_;
ros::Publisher point_cloud_publisher_;

// Constructor, gets number of sheep from automate script
sheepdogNode::sheepdogNode(int number) {
	this->sheepNum = number;
}

void sheepdogNode::StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 		
	
	// Set position
	px = 6 + msg.pose.pose.position.x;
	py = -0.5 + msg.pose.pose.position.y;
	tz = tz + msg.twist.twist.angular.z;

	// If the current position is the same the previous position, then the robot is stuck and needs to move around
	if ((px == prevpx) && (py == prevpy)) {
		if (!isRotate) {	
			ROS_INFO("Sheepdog stuck");
			double r2 = (double)rand()/((double)RAND_MAX/(M_PI/2));
			double m2 = (double)rand()/((double)RAND_MAX/0.5);
			move(-m2, r2);
			
		}	
	} 
	ROS_INFO("Sheepdog -- Current x position is: %f", px);
	ROS_INFO("Sheepdog -- Current y position is: %f", py);
	ROS_INFO("Sheepdog -- Current z position is: %f", tz);
	prevpx = px;
	prevpy = py;
}


// Send a velocity command
void sheepdogNode::move(double linearVelMPS, double angularVelRadPS) {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = linearVelMPS;
	msg.angular.z = angularVelRadPS;
	commandPub.publish(msg);
}




// Process the incoming laser scan message
void sheepdogNode::commandCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	sensor_msgs::PointCloud cloud;
	//projector_.transformLaserScanToPointCloud("robot_1/base_link", *msg, cloud, tfListener_);
	point_cloud_publisher_.publish(cloud);
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
		prevclosestRange = closestRange;
		if (closestRange > 20) {
			fsm=FSM_ROTATE;
			rotateStartTime=ros::Time::now();
			rotateDuration=ros::Duration(0.001);
		}
	}
}


// Process the incoming sheep position messages
void sheepdogNode::chaseSheepCallback(geometry_msgs::Pose2D msg) {
	ROS_INFO("I see a sheep at (%f, %f)",msg.x,msg.y);
	if(truckX != -1.0 && truckY != -1.0){
		// This loop executes when the truck has given its coordinates.
		// Don't want to do this unless we have them!
		
		// If the sheep dog is between the truck and the sheep furtherest away from the tuck
			// Walk to the other side of that sheep
		// Otherwise
			// Walk towards the truck
		
	}
}


// Store truck position
void sheepdogNode::truckCallback(geometry_msgs::Pose2D msg) {
	truckX = msg.x;
	truckY = msg.y;
}

// Main FSM loop for ensuring that ROS messages are
// processed in a timely manner, and also for sending
// velocity controls to the simulated robot based on the FSM state
void sheepdogNode::spin() {
	ros::Rate rate(10); // Specify the FSM loop rate in Hz
	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
		geometry_msgs::Pose2D msg;
		msg.x = px;
		msg.y = py;
		msg.theta = tz;
		
		if (fsm == FSM_MOVE_FORWARD) {
			move(FORWARD_SPEED_MPS, 0);
			checkcount++;
			if (checkcount > 3) {
				isRotate=false;
			}
		}
		
		if (fsm == FSM_ROTATE) {
			move(0, ROTATE_SPEED_RADPS);
			rotateEndTime=ros::Time::now();
			isRotate=true;
			if ((rotateEndTime - rotateStartTime) > rotateDuration) {
				fsm=FSM_MOVE_FORWARD;
				checkcount=0;
			}
		}
		
		sheepdogPosPub.publish(msg);
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
	}
}

	
	

void sheepdogNode::rosSetup(int argc, char **argv) {
	ros::init(argc, argv, "Sheepdog"); // Initiate new ROS node named "Sheepdog"
	ros::NodeHandle nh;
	tf::TransformListener tfListener_;
	ROS_INFO("This node is: Sheepdog");
	// Init FSM
	fsm = FSM(FSM_MOVE_FORWARD);
	// Initialize random time generator
	srand(time(NULL));
	// Setup Publishers and Subscribers
	// Cycle through sheep
	ros::Subscriber sheepPosSubs [sheepNum];
	for(int i = 0; i < sheepNum; i++){
		std::string current = "sheep_" + boost::lexical_cast<std::string>(i) + "/pose";
		sheepPosSubs[i] = nh.subscribe<geometry_msgs::Pose2D>(current, 1000, &sheepdogNode::chaseSheepCallback, this);
	}
	// Truck
	truckPosSub = nh.subscribe<geometry_msgs::Pose2D>("truck_position", 1000, &sheepdogNode::truckCallback, this);
	// Publish movement commands to stage
	commandPub = nh.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
	// Publish sheepdog position on sheepdog_postion topic
	sheepdogPosPub = nh.advertise<geometry_msgs::Pose2D>("sheepdog_position",1000);
	// Subscribe to the simulated robot's laser scan topic and tell ROS to call
	// this->commandCallback() whenever a new message is published on that topic
	laserSub = nh.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan", 1000, &sheepdogNode::commandCallback, this);
	StageOdo_sub = nh.subscribe<nav_msgs::Odometry>("robot_1/odom",1000, &sheepdogNode::StageOdom_callback,this);
	
	/*
	 * SUPER SECRET FUNCTION THAT DOESN'T QUITE WORK YET.
	 * Added a PointCloud generator to try and make an awesome 3D Dog Vision
	 * thing using rviz. Started working on this while waiting for sheep
	 * functionality to be added.
	 */
	point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud> ("/camera", 1000, false);
	tfListener_.setExtrapolationLimit(ros::Duration(0.1));
	
	prevpx = 6;
	prevpx= -0.5;
	this->spin();
}
