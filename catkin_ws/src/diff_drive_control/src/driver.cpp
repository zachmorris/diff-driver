#include "ros/ros.h"
// why is .msg file included as a .h?
#include "geometry_msgs/Twist.h"

#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "driver");
	
	// create node
  ros::NodeHandle n;

  // publish a message of type Twist on topic 'drive_direction' with buffer size = 1
  // why is <> outside the braces?
  // buffer size of 1 to reduce delayed commands
  ros::Publisher direction_pub = n.advertise<geometry_msgs::Twist>("drive_direction", 1);
	
	// define update frequency
  ros::Rate loop_rate(10);
  
  // what is the difference between ros.ok() and ros::ok()?
  while (ros::ok())
  {
		
    geometry_msgs::Twist vel;

    vel.linear.x = 0.5;
		
		// equivalent to printf/cout
    //ROS_INFO("%s", vel.linear.x.c_str());


    direction_pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
