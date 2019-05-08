// user includes
#include "driver.h"
#include "follow_wall.h"
#include "avoid_obstacles.h"
#include "go_to_goal.h"
#include "stop.h"

// ros includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "tf/tf.h"


// initialize robot goal pose (x, y, yaw)
Robot_Pose Robot_Goal = {3.0, 0.0, 0};

// diff_driver pose
Robot_Pose Current_Pose;

const double g_goal_distance = 0.2;
const double g_min_safe_distance = 0.5;


void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
	// get position
	Current_Pose.x = msg->pose.pose.position.x;
	Current_Pose.y = msg->pose.pose.position.y;
	ROS_INFO("Current position: [%f, %f]", Current_Pose.x, Current_Pose.y);	
	
	
	// convert quaternion to Euler angles, get yaw	
	tf::Quaternion q(
		msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, Current_Pose.yaw);
	
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "velocity_controller");
	
	// create node
  ros::NodeHandle node;
	
	// subscribe to odom with buffer size one to reduce delayed commands
	ros::Subscriber odom_sub = node.subscribe("odom", 1, odom_callback);
	
	// subscribe to scan with buffer size one to reduce delayed commands
	ros::Subscriber laser_sub = node.subscribe("scan", 1, laser_callback);
	
	// publish a message on cmd_vel with buffer size one
	ros::Publisher direction_pub;
  direction_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  
  // LUPE FIASCO
  ros::Rate rate(24.);
  
  // initialize robot state
	Robot_State diff_drive_state = Robot_States::GO_TO_GOAL;
  
  while (ros::ok()){
  	
  	switch (diff_drive_state){
  		case Robot_States::GO_TO_GOAL:
	  		if(dist_to_goal(Robot_Goal, Current_Pose) < g_goal_distance){
	  			ROS_INFO("Go to goal has reached the goal!");
	  			diff_drive_state = Robot_States::STOP;
	  		}
	  		else if (min_obstacle_distance() < g_min_safe_distance){
	  			ROS_INFO("Go to goal is avoiding an obstacle!");
	  			diff_drive_state = Robot_States::AVOID_OBSTACLE;
	  		}
	  		else {
	  			ROS_INFO("Go to goal!");
  				go_to_goal(direction_pub, Robot_Goal, Current_Pose);
	  		}  		
	  		break;
  		case Robot_States::AVOID_OBSTACLE:
  			if(dist_to_goal(Robot_Goal, Current_Pose) < g_goal_distance){
	  			ROS_INFO("Avoid obstacle has reached the");
	  			diff_drive_state = Robot_States::STOP;
	  		}
	  		else if (min_obstacle_distance() > g_min_safe_distance){
	  			ROS_INFO("Safe distance away, go to goal~");
	  			diff_drive_state = Robot_States::GO_TO_GOAL;
	  		}
	  		else {
	  			ROS_INFO("Avoid obstacle!");
  				avoid_obstacle(direction_pub, Current_Pose);
	  		}  		
  			break;
  		case Robot_States::STOP:
  			ROS_INFO("Stop!");
  			stop(direction_pub);
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
