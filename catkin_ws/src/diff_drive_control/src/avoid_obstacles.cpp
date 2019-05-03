// ros includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


// callback function for subscriber
void laser_callback(const sensor_msgs::LaserScan::ConstPtr &scan){
	// min_element returns pointer to the min element
	// dereference the pointer to get the min value
	float i1 = *std::min_element(std::begin(scan->ranges), std::end(scan->ranges)); 
  
  ROS_INFO("Closest element on right: [%f]", i1);			
}
