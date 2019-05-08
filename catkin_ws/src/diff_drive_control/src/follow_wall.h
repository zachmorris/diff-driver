#include <array>

// ros
#include "ros/ros.h"

double get_wall_heading(const std::array<float, 5> &dist_readings);

void follow_wall(ros::Publisher &direction_pub, Robot_Pose Current_Pose, const std::array<float, 5> &dist_readings);
