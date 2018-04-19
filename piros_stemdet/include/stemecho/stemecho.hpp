#pragma once

// ROS
#include <ros/ros.h>
#include "piros_stemdet/Stem.h"
#include "piros_stemdet/Stems.h"

namespace stemdet {
class Stemecho {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  Stemecho(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Stemecho();

  void stemCallback(const piros_stemdet::Stems& message);

 private:
  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  ros::NodeHandle nh_;

  std::string textSubscriberTopic_;

  ros::Subscriber stringSubscriber_;

  int count_;
};
};
