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
const double g_follow_wall_distance = 0.5;

// PID variables
const double g_kp 	= 1;


double get_wall_heading(const std::array<float, 5> &dist_readings, Robot_Pose Current_Pose){
	const double pi = 3.14159265358979323846;
	const double sensor_FOV = 3.14159;
  const int num_sensors = 5;
  const double sensor_spacing = sensor_FOV/num_sensors;
  const double start_angle = sensor_spacing/2 - pi/2;
  
	const double robot_chassis_length = 0.2;
  
  const double x_s = robot_chassis_length;
  const double y_s = 0;
  
  std::array<float, 5> dist_readings_wf_x;
  std::array<float, 5> dist_readings_wf_y;
  
  const double robot_yaw = Current_Pose.yaw;
  const double rx = Current_Pose.x;
  const double ry = Current_Pose.y;
  
	for(int i = 0; i < dist_readings.size(); ++i){		
		double dist_readings_rf_x = 0;
	  double dist_readings_rf_y = 0;
    double theta_s = start_angle + i*sensor_spacing;
    
    // transform distance from sensor frame to robot frame            
    dist_readings_rf_x = dist_readings[i]*cos(theta_s) + x_s;
    dist_readings_rf_y = dist_readings[i]*sin(theta_s) + y_s;
    
    // transform distances from robot frame to world frame            
    dist_readings_wf_x[i] = dist_readings_rf_x*cos(robot_yaw) - dist_readings_rf_y*sin(robot_yaw);
    dist_readings_wf_y[i] = dist_readings_rf_x*sin(robot_yaw) + dist_readings_rf_y*cos(robot_yaw);
	}
	
	/* FOLLOW WALL */
	
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
	
	double wall_front_x = dist_readings_wf_x[i];
	double wall_front_y = dist_readings_wf_y[i];
	//ROS_INFO("FW: Wall front coordinates: [%f, %f]", wall_front_x, wall_front_y);

	double wall_back_x = dist_readings_wf_x[j];
	double wall_back_y = dist_readings_wf_y[j];
	//ROS_INFO("FW: Wall back coordinates: [%f, %f]", wall_back_x, wall_back_y);
	
	// wall vector = FB (points from back to front, along direction of motion)
	double along_wall_x = wall_front_x - wall_back_x;
	double along_wall_y = wall_front_y - wall_back_y;
	double norm_along_wall = sqrt(pow(along_wall_x, 2.0) + pow(along_wall_y, 2.0));
	
  // length of wall^2 = ||FB||^2
	double ds = pow(along_wall_x, 2.0) + pow(along_wall_y, 2.0);
	
	// Theorem: shortest distance from point R to line AB is ||RA x RB|| / ||AB||
	// ||AB|| == ds
	// In 2D, ||RA x RB|| == RA x RB == RFront_x * RBack_y - RFront_y * RBack_x
	double ms = -wall_back_x * along_wall_y + wall_back_y * along_wall_x;; 
	// double ms = wall_front_x * wall_back_y - wall_front_y * wall_back_x;
	
  // Rotate wall vector 90 degrees so it points to the wall: (x, y) --> (-y, x)
  // FB_rot / ||FB|| is a unit vector pointing to the wall
  // ||RF x RB|| * FB_rot / ||FB||^2
  double to_wall_x = (wall_back_y - wall_front_y)*ms/ds;
  double to_wall_y = (wall_front_x - wall_back_x)*ms/ds;
  double norm_to_wall = sqrt(pow(to_wall_x, 2.0) + pow(to_wall_y, 2.0));
  //ROS_INFO("FW: to wall coordinates: [%f, %f]", to_wall_x, to_wall_y);
  //ROS_INFO("FW: Distance to wall: [%f]", norm_to_wall);
     
  // heading vector
	double offset = fabs(ms/sqrt(ds)) - g_follow_wall_distance;
	//ROS_INFO("FW: offset [%f]", offset);
	
	heading_x = g_follow_wall_distance * along_wall_x/norm_along_wall + offset * to_wall_x/norm_to_wall;
	heading_y = g_follow_wall_distance * along_wall_y/norm_along_wall + offset * to_wall_y/norm_to_wall;		
	fw_heading = atan2(heading_y, heading_x);
	
	
	//ROS_INFO("wall follow heading: (%f)", fw_heading);
	return fw_heading;
}


void follow_wall(ros::Publisher &direction_pub, Robot_Pose Current_Pose, const std::array<float, 5> &dist_readings){
	double desired_yaw, err_yaw;
	desired_yaw = get_wall_heading(dist_readings, Current_Pose);
	// angles::shortest_angular_distance(from, to);
	err_yaw = angles::shortest_angular_distance(Current_Pose.yaw, desired_yaw);	
	
	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
		//ROS_INFO("FW: Turning parallel to wall.");
		vel.angular.z = -g_kp * err_yaw;
		//ROS_INFO("FW: Current yaw: [%f]", Current_Pose.yaw);
		//ROS_INFO("FW: Desired yaw: [%f]", desired_yaw);		
		//ROS_INFO("Current error: [%f]", err_yaw);				
		
	} else {
		//ROS_INFO("FW: Parallel to wall.");	
		vel.angular.z = 0;		
	}
		
	vel.linear.x = 0.1;		
		
	direction_pub.publish(vel);

}
