#include <ros/ros.h>
#include "vegsec_handler.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "vegsec_node");
  ros::NodeHandle node("~");

  vegsec::VegSecHandler vegetation_handler(node);

  ros::spin();
  return 0;
}
