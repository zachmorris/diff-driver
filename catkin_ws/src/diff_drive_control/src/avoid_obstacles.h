// ros
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

//void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

//double min_obstacle_distance();

double get_avoidance_heading(const std::array<float, 5> &regions);

void avoid_obstacle(ros::Publisher &direction_pub, Robot_Pose Current_Pose, const std::array<float, 5> &regions);
