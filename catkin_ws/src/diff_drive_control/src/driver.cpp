#include "ros/ros.h"
// why is .msg file included as a .h?
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"

#include <sstream>


// callback function for subscriber
void callback(const nav_msgs::Odometry::ConstPtr &msg){
	// get position
	double x, y;
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	
	
	// convert quaternion to Euler angles, get yaw	
	tf::Quaternion q(
		msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	
	// do something with yaw
	ROS_INFO("Current yaw: [%f]", yaw);

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "velocity_controller");
	
	// create node
  ros::NodeHandle node;
	
	// subscribe to odom with buffer size one to reduce delayed commands
	ros::Subscriber sub = node.subscribe("odom", 1, callback);
	
	// publish a message on cmd_vel with buffer size one
  ros::Publisher direction_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
	ros::spin();

  return 0;
}
