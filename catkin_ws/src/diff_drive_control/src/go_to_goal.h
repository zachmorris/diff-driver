// ros
#include "ros/ros.h"

double get_goal_heading(Robot_Pose Robot_Goal, Robot_Pose Current_Pose);

void go_to_goal(ros::Publisher &direction_pub, Robot_Pose Robot_Goal, Robot_Pose Current_Pose);

double dist_to_goal(Robot_Pose Robot_Goal, Robot_Pose Current_Pose);
