#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mvg_localizer_node");
  ros::NodeHandle n;
  ROS_INFO("Entered node!");
  ros::spin();
  return 0;
}