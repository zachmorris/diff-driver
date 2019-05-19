// user includes
#include "driver.h"
#include "follow_wall.h"
#include "avoid_obstacles.h"
#include "go_to_goal.h"
#include "stop.h"

// c++ includes
#include <array>

// ros includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"


// initialize robot goal pose (x, y, yaw)
Robot_Pose Robot_Goal = {3.0, 2.0, 0};

// diff_driver pose
Robot_Pose Current_Pose;

const double g_goal_distance = 0.2;
const double g_min_safe_distance = 0.4;
const double g_min_fw_distance = 0.6;

double g_closest_to_goal;


// laser scan variables
double min_dist;
std::array<float, 5> regions;


void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
	// get position
	Current_Pose.x = msg->pose.pose.position.x;
	Current_Pose.y = msg->pose.pose.position.y;
	//ROS_INFO("Current position: [%f, %f]", Current_Pose.x, Current_Pose.y);	
	
	
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
  //ROS_INFO("Closest element: [%f]", min_dist);
}


double min_obstacle_distance(){
	return min_dist;
}


bool progress_made(){
	ROS_INFO("Distance to goal: (%f)", dist_to_goal(Robot_Goal, Current_Pose));
	ROS_INFO("Closest so far: (%f)", g_closest_to_goal);
	return (dist_to_goal(Robot_Goal, Current_Pose) < (g_closest_to_goal - 0.01));
}


bool can_detach(){
	ROS_INFO("Goal heading: (%f)", get_goal_heading(Robot_Goal, Current_Pose));
	ROS_INFO("Wall heading: (%f)", get_wall_heading(regions));
	
	return (get_goal_heading(Robot_Goal, Current_Pose) < get_wall_heading(regions));
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
	  			ROS_INFO("GTG --> AO");
	  			g_closest_to_goal = dist_to_goal(Robot_Goal, Current_Pose);
	  			diff_drive_state = Robot_States::AVOID_OBSTACLE;
	  		}
	  		else {
  				go_to_goal(direction_pub, Robot_Goal, Current_Pose);
	  		}  		
	  		break;
	  		
  		case Robot_States::AVOID_OBSTACLE:
  			if(dist_to_goal(Robot_Goal, Current_Pose) < g_goal_distance){
	  			ROS_INFO("Avoid obstacle is at the goal!");
	  			diff_drive_state = Robot_States::STOP;
	  		}
	  		else if (min_obstacle_distance() > g_min_fw_distance){
	  			ROS_INFO("Avoid obstacle going to goal");
	  			diff_drive_state = Robot_States::GO_TO_GOAL;
	  		}
	  		else {
  				avoid_obstacle(direction_pub, Current_Pose, regions);
	  		}  		
  			break;
  			
  		case Robot_States::FOLLOW_WALL:
  			if(dist_to_goal(Robot_Goal, Current_Pose) < g_goal_distance){
	  			ROS_INFO("Follow wall has reached the goal");
	  			diff_drive_state = Robot_States::STOP;
	  		}
	  		else if (progress_made() and can_detach()){
	  			ROS_INFO("Can detach from wall and go to goal");
	  			diff_drive_state = Robot_States::GO_TO_GOAL;
	  		}
	  		else if (min_obstacle_distance() < g_min_safe_distance){
	  			ROS_INFO("FW --> AO");
	  			diff_drive_state = Robot_States::AVOID_OBSTACLE;
	  		}
	  		else {
  				follow_wall(direction_pub, Current_Pose, regions);
	  		}  		
  			break;

  		case Robot_States::STOP:
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
