// user
#include "driver.h" 	// for robot states
#include "avoid_obstacles.h"

// c++
#include <cmath>			// for fabs

// ros includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


// precision
const double g_yaw_precision 	= 3.14159/90;
const double g_min_safe_distance = 0.5;

// PID variables
const double g_kp 	= 0.5;

// laser scan variables
double min_dist;
std::array<float, 5> regions;


void laser_callback(const sensor_msgs::LaserScan::ConstPtr &scan){
	
 regions = { 
		*std::min_element((scan->ranges).begin()      , (scan->ranges).begin() + 143),
		*std::min_element((scan->ranges).begin() + 144, (scan->ranges).begin() + 287),
		*std::min_element((scan->ranges).begin() + 288, (scan->ranges).begin() + 431),
		*std::min_element((scan->ranges).begin() + 432, (scan->ranges).begin() + 575),
		*std::min_element((scan->ranges).begin() + 576, (scan->ranges).begin() + 719)
	};

  for(auto &element : regions){
  	if(std::isinf(element))
  		element = 10;			
  }
  
  min_dist = *std::min_element(regions.begin(), regions.end());
  ROS_INFO("Closest element: [%f]", min_dist);
}


double min_obstacle_distance(){
	return min_dist;
}

int get_avoidance_heading(){
	// cos and sin of 18, 54, 90, 126, 162. I know, there's a better way to do this 
	std::array<float, 5> cos = {0.95105691, 0.58778832, 0.00000633, -0.58777809, -0.95105300};
	std::array<float, 5> sin = {0.3090157909, 0.8090147631, 1, 0.8090222007, 0.3090278252};
		
	return 0;
}

Robot_State avoid_obstacle(ros::Publisher &direction_pub, Robot_Pose Current_Pose){
	Robot_State diff_drive_state = Robot_States::AVOID_OBSTACLE;
	
	return diff_drive_state;
};/*
	Robot_State diff_drive_state = Robot_States::AVOID_OBSTACLE;

	double desired_yaw, err_yaw;
	desired_yaw = get_avoidance_heading();
	err_yaw = desired_yaw - Current_Pose.yaw;
	
	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
		ROS_INFO("Correcting yaw.");
		vel.angular.z = -g_kp * err_yaw;
		ROS_INFO("Current yaw: [%f]", Current_Pose.yaw);
		ROS_INFO("Desired yaw: [%f]", desired_yaw);		
		ROS_INFO("Current error: [%f]", err_yaw);				
		
	} else {
		ROS_INFO("Yaw good homes.");	
		vel.angular.z = 0;		
	}
	
	if (fabs(dist_to_obstacle) < g_min_safe_distance){
		ROS_INFO("Driving away from obstacle.");
		vel.linear.x = 0.1;				
		
	} else {
		ROS_INFO("Position good homes.");			
		diff_drive_state = Robot_State::GO_TO_GOAL;
	}
		
	direction_pub.publish(vel);
	
	return diff_drive_state;
}*/
