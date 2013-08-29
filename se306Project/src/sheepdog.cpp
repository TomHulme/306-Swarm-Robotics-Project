#include <se306Project/sheepdog.h>

const static double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
const static double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
const static float PROXIMITY_RANGE_M = 5;
const static double FORWARD_SPEED_MPS = 1;
const static double ROTATE_SPEED_RADPS = M_PI;

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
int sheepNum = 0;
int checkcount = 0;
enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE};
ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
ros::Publisher sheepdogPosPub;
ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
enum FSM fsm; // Finite state machine for the random walk algorithm

ros::Time rotateStartTime; // Start time of the rotation
ros::Time rotateEndTime;
ros::Duration rotateDuration; // Duration of the rotation
ros::Subscriber StageOdo_sub;

laser_geometry::LaserProjection projector_;

ros::Publisher point_cloud_publisher_;


sheepdogNode::sheepdogNode(int number) {
	sheepNum = number;
}

void sheepdogNode::StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 		

	px = 5 + msg.pose.pose.position.x;
	py = 5 + msg.pose.pose.position.y;
	tz = tz + msg.twist.twist.angular.z;
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
			ROS_INFO("Sheepdog stuck");
			double r2 = (double)rand()/((double)RAND_MAX/(M_PI/2));
			double m2 = (double)rand()/((double)RAND_MAX/0.5);
			move(-m2, r2);
			
		}	
	} else {
		// One the robot becomes unstuck, then it moves around again normally
		//linear_x = 0.2;
		//angular_z = 0.2;
		//ROS_INFO("Robot unstuck");
		//checkcount=0;
	}
	ROS_INFO("Sheepdog -- Current x position is: %f", px);
	ROS_INFO("Sheepdog -- Current y position is: %f", py);
	ROS_INFO("Sheepdog -- Current z position is: %f", tz);
	ROS_INFO("Sheepdog -- Current sheepNum is: %d", sheepNum);
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
	projector_.transformLaserScanToPointCloud("robot_1/base_link", *msg, cloud, tfListener_);
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
		
		
		//if (closestRange == prevclosestRange) {
		//	ROS_INFO_STREAM("STUCK");
		//	move(-FORWARD_SPEED_MPS, ROTATE_SPEED_RADPS);
		//	//move(0, ROTATE_SPEED_RADPS);
		//} else {
			
			//ROS_INFO_STREAM("Range: " << closestRange);
			prevclosestRange = closestRange;
			if (closestRange > 20) {
				fsm=FSM_ROTATE;
				rotateStartTime=ros::Time::now();
				rotateDuration=ros::Duration(0.001);
				//fsm= FSM_MOVE_FORWARD;
			}
		//}

	}
}




// Process the incoming sheep position messages
void sheepdogNode::chaseSheepCallback(geometry_msgs::Pose2D msg) {
	if (fsm == FSM_MOVE_FORWARD) {
		

	}
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
	fsm = FSM(FSM_MOVE_FORWARD);
	// Initialize random time generator
	srand(time(NULL));
	// Advertise a new publisher for the simulated robot's velocity command topic
	// (the second argument indicates that if multiple command messages are in
	// the queue to be sent, only the last command will be sent)
	std::string sheepPosSubs [sheepNum];
	std::ostringstream buffer;
	for(int i = 0; i < sheepNum; i++){
		buffer << i;
		nh.subscribe<geometry_msgs::Pose2D>("sheep_" + buffer.str() + "/pose", 1000, &sheepdogNode::chaseSheepCallback, this);
	}
	commandPub = nh.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
	sheepdogPosPub = nh.advertise<geometry_msgs::Pose2D>("sheepdog_position",1000);
	// Subscribe to the simulated robot's laser scan topic and tell ROS to call
	// this->commandCallback() whenever a new message is published on that topic
	laserSub = nh.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan", 1000, &sheepdogNode::commandCallback, this);
	StageOdo_sub = nh.subscribe<nav_msgs::Odometry>("robot_1/odom",1000, &sheepdogNode::StageOdom_callback,this);
	point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud> ("/camera", 1000, false);
	tfListener_.setExtrapolationLimit(ros::Duration(0.1));
	
	prevpx = 5;
	prevpx= 5;
	this->spin();
}
