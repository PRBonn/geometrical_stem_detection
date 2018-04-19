#include "stemdetection/stemdetection.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "stemdetection_node");
  ros::NodeHandle nodeHandle("~");

  stemdet::Stemdetection Stemdetection(nodeHandle);

  ros::spin();
  return 0;
}
