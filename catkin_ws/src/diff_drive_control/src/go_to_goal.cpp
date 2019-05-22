// user
#include "driver.h" 	// for robot states
#include "go_to_goal.h"

// c++
#include <cmath>			// for fabs

// ros
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"


// precision
const double g_yaw_precision 	= 3.14159/90;
//const double g_dist_precision = 0.2; moved to driver.cpp

// PID variables
const double g_kp 	= 0.5;


double get_goal_heading(Robot_Pose Robot_Goal, Robot_Pose Current_Pose){
	return atan2(Robot_Goal.y - Current_Pose.y, Robot_Goal.x - Current_Pose.x);
}

void go_to_goal(ros::Publisher &direction_pub, Robot_Pose Robot_Goal, Robot_Pose Current_Pose){
	//Robot_State diff_drive_state = Robot_States::GO_TO_GOAL;

	double desired_yaw, err_yaw;
	desired_yaw = get_goal_heading(Robot_Goal, Current_Pose);
	err_yaw = desired_yaw - Current_Pose.yaw;
	
	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
		//ROS_INFO("GTG: Correcting yaw.");
		vel.angular.z = -g_kp * err_yaw;
		//ROS_INFO("GTG: Current yaw: [%f]", Current_Pose.yaw);
		//ROS_INFO("GTG: Desired yaw: [%f]", desired_yaw);		
		//ROS_INFO("Current error: [%f]", err_yaw);				
		
	} else {
		//ROS_INFO("GTG: Yaw good homes.");	
		vel.angular.z = 0;		
	}
	vel.linear.x = 0.2;		
		
	direction_pub.publish(vel);
	
	//return diff_drive_state;
}


double dist_to_goal(Robot_Pose Robot_Goal, Robot_Pose Current_Pose){
	double dist = sqrt(pow(Robot_Goal.y - Current_Pose.y, 2.0) + pow(Robot_Goal.x - Current_Pose.x, 2.0));
	//ROS_INFO("Distance to goal: [%f]", dist);
	
	return fabs(dist);
}
