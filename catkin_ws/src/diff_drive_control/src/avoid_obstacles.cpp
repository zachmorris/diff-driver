// user
#include "driver.h" 	// for robot states
#include "avoid_obstacles.h"

// c++
#include <cmath>			// for fabs
#include <array>

// ros includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


// precision
const double g_yaw_precision 	= 3.14159/90;

// PID variables
const double g_kp 	= 0.5;


double get_avoidance_heading(const std::array<float, 5> &regions){
	// cos and sin of 18, 54, 90, 126, 162. I know, there's a better way to do this 
	std::array<float, 5> cos = {0.95105691, 0.58778832, 0, -0.58777809, -0.95105300};
	std::array<float, 5> sin = {0.3090157909, 0.8090147631, 1, 0.8090222007, 0.3090278252};
	
	double x_sum = 0;
	double y_sum = 0;
	
	for(int i = 0; i < regions.size(); ++i){
		x_sum = x_sum + regions[i] * cos[i];
		y_sum = y_sum + regions[i] * sin[i];
	}
	
	double desired_yaw = atan2(y_sum, x_sum);
		
	return desired_yaw;
}

void avoid_obstacle(ros::Publisher &direction_pub, Robot_Pose Current_Pose, const std::array<float, 5> &regions){
	double desired_yaw, err_yaw;
	desired_yaw = get_avoidance_heading(regions);
	err_yaw = desired_yaw - Current_Pose.yaw;
	
	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
		//ROS_INFO("Turning away from obstacle.");
		vel.angular.z = -g_kp * err_yaw;
		//ROS_INFO("Current yaw: [%f]", Current_Pose.yaw);
		//ROS_INFO("Desired yaw: [%f]", desired_yaw);		
		//ROS_INFO("Current error: [%f]", err_yaw);				
		
	} else {
		//ROS_INFO("Pointing away from the obstacle.");	
		vel.angular.z = 0;		
	}
	vel.linear.x = 0.2;		
		
	direction_pub.publish(vel);

}
