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
#include "angles/angles.h"


// precision
const double g_yaw_precision 	= 3.14159/90;

// PID variables
const double g_kp 	= 0.5;


double get_avoidance_heading(const std::array<float, 5> &laser_dist, double robot_yaw){
	const double pi = 3.14159265358979323846;
	const double sensor_FOV = 3.14159;
  const int num_sensors = 5;
  const double sensor_spacing = sensor_FOV/num_sensors;
  const double start_angle = sensor_spacing/2 - pi/2;
  
	const double robot_chassis_length = 0.2;
   
  double laser_dist_wf_x = 0; 
  double laser_dist_wf_y = 0;
  
  const double x_s = robot_chassis_length;
  const double y_s = 0;
  
	for(int i = 0; i < laser_dist.size(); ++i){		
		double laser_dist_rf_x = 0;
	  double laser_dist_rf_y = 0;
    double theta_s = start_angle + i*sensor_spacing;
    
    // transform distance from sensor frame to robot frame            
    laser_dist_rf_x = laser_dist[i]*cos(theta_s) + x_s;
    laser_dist_rf_y = laser_dist[i]*sin(theta_s) + y_s;
    
    // transform distances from robot frame to world frame            
    laser_dist_wf_x = laser_dist_wf_x + laser_dist_rf_x*cos(robot_yaw) - laser_dist_rf_y*sin(robot_yaw);
    laser_dist_wf_y = laser_dist_wf_y + laser_dist_rf_x*sin(robot_yaw) + laser_dist_rf_y*cos(robot_yaw);
	}
	
	double turn_left = 0.75/(0.5 + 1*laser_dist[2]);
	
	double desired_yaw = atan2(laser_dist_wf_y, laser_dist_wf_x) + turn_left;
		
	return desired_yaw;
}

void avoid_obstacle(ros::Publisher &direction_pub, Robot_Pose Current_Pose, const std::array<float, 5> &regions){
	double desired_yaw, err_yaw;
	desired_yaw = get_avoidance_heading(regions, Current_Pose.yaw);
	//err_yaw = desired_yaw - Current_Pose.yaw;
	err_yaw = angles::shortest_angular_distance(Current_Pose.yaw, desired_yaw);	
	
	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
		//ROS_INFO("Turning away from obstacle.");
		vel.angular.z = -g_kp * err_yaw;
		ROS_INFO("Current yaw: [%f]", Current_Pose.yaw);
		ROS_INFO("Desired yaw: [%f]", desired_yaw);		
		ROS_INFO("Current error: [%f]", err_yaw);				
		
	} else {
		//ROS_INFO("Pointing away from the obstacle.");	
		vel.angular.z = 0;		
	}
	vel.linear.x = 0.1;		
		
	direction_pub.publish(vel);

}
