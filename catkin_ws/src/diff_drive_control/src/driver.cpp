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
const int g_desired_position_x = -3;
const int g_desired_position_y = 7;

// precision
const double g_yaw_precision = 3.14159/90;
const double g_dist_precision = 0.3;


ros::Publisher direction_pub;



// callback function for subscriber
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
	// get position
	g_x = msg->pose.pose.position.x;
	g_y = msg->pose.pose.position.y;
	
	
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
	ROS_INFO("Current yaw: [%f]", g_yaw);
	
}


void go_to_goal(){
	double desired_yaw, err_yaw;
	desired_yaw = atan2(g_desired_position_y - g_y, g_desired_position_x - g_x);
	
	err_yaw = desired_yaw - g_yaw;
	
	if (fabs(err_yaw) > g_yaw_precision){
		ROS_INFO("Correcting yaw.");
		// TODO add twist message, fill with delicious yaw and publish
	} else {
		ROS_INFO("Yaw good homes.");
	}
		
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
  	
  	go_to_goal();
  	
		ros::spinOnce();
		rate.sleep();
  }
  
  
  return 0;
}
