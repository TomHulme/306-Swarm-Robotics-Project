#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "se306Project/SheepMoveMsg.h"


#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

enum EyeType {
  GROUND_VIEW,
  GRASS_VIEW,
  MAIN_VIEW
};

class SheepVision {

      	public:
        EyeType robotType;
        int sheepNum; 
        
}