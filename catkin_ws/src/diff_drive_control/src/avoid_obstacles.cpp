// ros includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


// callback function for subscriber
void laser_callback(const sensor_msgs::LaserScan::ConstPtr &scan){
	
	std::array<float, 5> regions { 
		*std::min_element((scan->ranges).begin()      , (scan->ranges).begin() + 143),
		*std::min_element((scan->ranges).begin() + 144, (scan->ranges).begin() + 287),
		*std::min_element((scan->ranges).begin() + 288, (scan->ranges).begin() + 431),
		*std::min_element((scan->ranges).begin() + 432, (scan->ranges).begin() + 575),
		*std::min_element((scan->ranges).begin() + 576, (scan->ranges).begin() + 719)
	};

  for(auto &element : regions){
  	if(std::isinf(element))
  		element = 10;
  	
    ROS_INFO("Closest element on right: [%f]", element);			
  }
}
