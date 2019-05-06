// ros
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

double min_obstacle_distance();

int get_avoidance_heading();

Robot_State avoid_obstacle(ros::Publisher &direction_pub);
