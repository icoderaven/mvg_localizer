/**
 * Node to try and localize new images given a precomputed databse in MVG
 */
#include <ros/ros.h>

//MVG imports
#include "openMVG/sfm/sfm.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace openMVG;
using namespace openMVG::sfm;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mvg_localizer_node");
  ros::NodeHandle n;
  ROS_INFO("Entered node!");
  ros::spin();
  return 0;
}
