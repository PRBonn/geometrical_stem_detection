#include "stemecho/stemecho.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "stemecho_node");

  ros::NodeHandle nodeHandle("~");

  stemdet::Stemecho echo(nodeHandle);

  ros::spin();

  return 0;
}
