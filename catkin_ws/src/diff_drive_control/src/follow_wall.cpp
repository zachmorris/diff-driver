// user
#include "driver.h" 	// for robot states
#include "follow_wall.h"

// c++
#include <cmath>			// for fabs
#include <array>

// ros includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "angles/angles.h"


// precision
const double g_yaw_precision 	= 3.14159/90;
const double g_follow_wall_distance = 0.4;

// PID variables
const double g_kp 	= 1;


double get_wall_heading(const std::array<float, 5> &dist_readings){
	
	int i = 0, j = 0;
	double heading_x = 0, heading_y = 0;
	double fw_heading = 0;
	
	// we use the sensors on the RIGHT of the robot to travel LEFT along the wall
	// decide whether to use 3PM & 1PM sensor, or 1PM and noon sensor
	auto min_it = std::min_element(dist_readings.begin(), dist_readings.end());
	int min_in = std::distance(dist_readings.begin(), min_it);
	
	if(min_in == 0 or (min_in == 1 and dist_readings[0] <= dist_readings[2])){
		i = 1;
		j = 0;
		//ROS_INFO("Sensors 1 and 0 closest to wall.");
	}
	else {
		i = 2;
		j = 1;
		//ROS_INFO("Sensors 2 and 1 closest to wall.");
	}
	
	
	// find vector
	// cos and sin of 18, 54, 90, 126, 162. I know, there's a better way to do this 
	std::array<float, 5> cos = {0.95105691, 0.58778832, 0, -0.58777809, -0.95105300};
	std::array<float, 5> sin = {0.3090157909, 0.8090147631, 1, 0.8090222007, 0.3090278252};
	
	// Note that the sensors are defined relative to R = (0,0)
	double wall_front_x = dist_readings[i] * cos[i];
	double wall_front_y = dist_readings[i] * sin[i];
	
	//ROS_INFO("wall_front_x: (%f)", wall_front_x);
	//ROS_INFO("wall_front_y: (%f)", wall_front_y);	

	double wall_back_x = dist_readings[j] * cos[j];
	double wall_back_y = dist_readings[j] * sin[j];
	
	// wall vector = FB (points from back to front, along direction of motion)
	double along_wall_x = wall_front_x - wall_back_x;
	double along_wall_y = wall_front_y - wall_back_y;
	
	//ROS_INFO("along_wall_x: (%f)", along_wall_x);
	//ROS_INFO("along_wall_y: (%f)", along_wall_y);	
	
  // ||FB||^2
	double ds = pow(along_wall_x, 2.0) + pow(along_wall_y, 2.0);
	
	// Theorem: shortest distance from point R to line AB is ||RA x RB|| / ||AB||
	// ||AB|| == ds
	// In 2D, ||RA x RB|| == RA x RB == RFront_x * RBack_y - RFront_y * RBack_x
	double ms = wall_front_x * wall_back_y - wall_front_y * wall_back_x;
	
	//ROS_INFO("ds: (%f)", ds);
	//ROS_INFO("ms: (%f)", ms);	
	
  // Rotate wall vector 90 degrees so it points to the wall: (x, y) --> (-y, x)
  // FB_rot / ||FB|| is a unit vector pointing to the wall
  // ||RF x RB|| * FB_rot / ||FB||^2
  double to_wall_x = (wall_back_y - wall_front_y)*ms/ds;
  double to_wall_y = (wall_front_x - wall_back_x)*ms/ds;
     
  // heading vector
	double offset = fabs(ms/sqrt(ds)) - g_follow_wall_distance;
	
	if (offset > 0){
		heading_x = 0.3 * along_wall_x + 2 * offset * to_wall_x;
		heading_y = 0.3 * along_wall_y + 2 * offset * to_wall_y;		
		fw_heading = atan2(heading_y, heading_x);
	}
	else {
		heading_x = 0.3 * along_wall_x + 3 * offset * to_wall_x;
		heading_y = 0.3 * along_wall_y + 3 * offset * to_wall_y;		
		fw_heading = atan2(heading_y, heading_x);	
	}
	
	ROS_INFO("wall follow heading: (%f)", fw_heading);
	return fw_heading;
}


void follow_wall(ros::Publisher &direction_pub, Robot_Pose Current_Pose, const std::array<float, 5> &dist_readings){
	double desired_yaw, err_yaw;
	desired_yaw = get_wall_heading(dist_readings);
	// angles::shortest_angular_distance(from, to);
	err_yaw = angles::shortest_angular_distance(Current_Pose.yaw, desired_yaw);	
	
	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
		ROS_INFO("Turning parallel to wall.");
		vel.angular.z = -g_kp * err_yaw;
		ROS_INFO("Current yaw: [%f]", Current_Pose.yaw);
		ROS_INFO("Desired yaw: [%f]", desired_yaw);		
		ROS_INFO("Current error: [%f]", err_yaw);				
		
	} else {
		ROS_INFO("Parallel to wall.");	
		vel.angular.z = 0;		
	}
		
	vel.linear.x = 0.15;		
		
	direction_pub.publish(vel);

}
