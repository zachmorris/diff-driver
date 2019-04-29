// ros
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"

void stop(ros::Publisher &direction_pub){
	geometry_msgs::Twist vel;
	
	vel.linear.x = 0;
	vel.angular.z = 0;
	
	direction_pub.publish(vel);	
}

