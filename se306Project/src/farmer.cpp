#include <se306Project/farmer.h>


farmerNode::farmerNode(int xi, int yi){
	
	//initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	this->theta = M_PI/2.0;
	this->px = xi;
	this->py = yi;
	this->prevpx = 0;
	this->prevpx= 0;
	
	//Initial velocity
	this->linear_x = 0.0;
	this->angular_z = 0.2;
}

void farmerNode::StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 		
	
	this->px = 5 + msg.pose.pose.position.x;
	this->py =10 + msg.pose.pose.position.y;
	
	// If the current position is the same the previous position, then the robot is stuck and needs to move around
	if ((this->px == this->prevpx) && (this->py == this->prevpy)) {

		// Note the negative linear_x		
		this->linear_x=-0.0;
		this->angular_z=1;

	} else {
		// One the robot becomes unstuck, then it moves around again normally
		this->linear_x = 0.0;
		this->angular_z = 1;
	}
	ROS_INFO("Farmer -- Current x position is: %f", this->px);
	ROS_INFO("Farmer -- Current y position is: %f", this->py);
	this->prevpx = this->px;
	this->prevpy = this->py;
}


void farmerNode::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	// Farmer currently has no need for his laser. But he might, so keep this.
	
}

void farmerNode::rosSetup(int argc, char **argv)
{
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Farmer");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &farmerNode::StageOdom_callback, this);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000, &farmerNode::StageLaser_callback, this);

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
			
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

}
