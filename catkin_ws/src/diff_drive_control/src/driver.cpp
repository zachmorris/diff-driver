// c++ includes
#include <cmath>	// for fabs

// ros includes
#include "ros/ros.h"
// why is .msg file included as a .h?
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"



#include <sstream>

double g_x, g_y, g_yaw;

// goal
const int g_x_goal = -2;
const int g_y_goal = 3;

// precision
const double g_yaw_precision = 3.14159/90;
const double g_dist_precision = 0.2;

// PID variables
const double g_kp = 0.5;

namespace Robot_States
{
  enum Robot_State
  {
     GO_TO_GOAL,
     STOP
  };
}
typedef Robot_States::Robot_State Robot_State;


Robot_State diff_drive_state = Robot_States::GO_TO_GOAL;

ros::Publisher direction_pub;



// callback function for subscriber
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
	// get position
	g_x = msg->pose.pose.position.x;
	g_y = msg->pose.pose.position.y;
	ROS_INFO("Current position: [%f, %f]", g_x, g_y);	
	
	
	// convert quaternion to Euler angles, get yaw	
	tf::Quaternion q(
		msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, g_yaw);
	
	// do something with yaw
	//ROS_INFO("Current yaw: [%f]", g_yaw);
	
}


void go_to_goal(){
	double desired_yaw, err_yaw;
	desired_yaw = atan2(g_y_goal - g_y, g_x_goal - g_x);
	
	err_yaw = desired_yaw - g_yaw;
	
	geometry_msgs::Twist vel;		
	
	if (fabs(err_yaw) > g_yaw_precision){
		ROS_INFO("Correcting yaw.");
		vel.angular.z = -g_kp * err_yaw;
		ROS_INFO("Current yaw: [%f]", g_yaw);
		ROS_INFO("Desired yaw: [%f]", desired_yaw);		
		ROS_INFO("Current error: [%f]", err_yaw);				
		
	} else {
		ROS_INFO("Yaw good homes.");	
		vel.angular.z = 0;		
	}
	
	//TODO fix this position thing
	double dist_to_goal = sqrt(pow(g_y_goal - g_y, 2.0) + pow(g_x_goal - g_x, 2.0));
	
	ROS_INFO("Distance to goal: [%f]", dist_to_goal);
	
	if (fabs(dist_to_goal) > g_dist_precision){
		ROS_INFO("Driving forward.");
		vel.linear.x = 0.1;				
		
	} else {
		ROS_INFO("Position good homes.");			
		diff_drive_state = Robot_State::STOP;
	}
		
	direction_pub.publish(vel);
}


void stop(){
	geometry_msgs::Twist vel;
	
	vel.linear.x = 0;
	vel.angular.z = 0;
	
	direction_pub.publish(vel);	
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "velocity_controller");
	
	// create node
  ros::NodeHandle node;
	
	// subscribe to odom with buffer size one to reduce delayed commands
	ros::Subscriber odom_sub = node.subscribe("odom", 1, odom_callback);
	
	// publish a message on cmd_vel with buffer size one
  direction_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  
  // LUPE FIASCO
  ros::Rate rate(24.);
  
  while (ros::ok()){
  	
  	switch (diff_drive_state){
  		case Robot_States::GO_TO_GOAL:
	  		ROS_INFO("Go to goal!");
  			go_to_goal();
  			break;
  		case Robot_States::STOP:
  			ROS_INFO("Stop!");
  			stop();
  			break;
  		default:
  			ROS_INFO("Unknown state!");
  			break;
  	}
  	
		ros::spinOnce();
		rate.sleep();
  }
  
  
  return 0;
}
