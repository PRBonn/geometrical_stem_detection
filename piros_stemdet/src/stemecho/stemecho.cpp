#include <sstream>
#include <string>

#include "stemecho/stemecho.hpp"

namespace stemdet {
Stemecho::Stemecho(ros::NodeHandle& nodeHandle) : nh_(nodeHandle) {
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  stringSubscriber_ =
      nh_.subscribe(textSubscriberTopic_, 1, &Stemecho::stemCallback, this);

  count_ = 0;
}

Stemecho::~Stemecho() {}

bool Stemecho::readParameters() {
  if (!nh_.getParam("text_subscriber_topic", textSubscriberTopic_))
    return false;
  return true;
}

void Stemecho::stemCallback(const piros_stemdet::Stems& message) {
  ROS_INFO("Received Stems: ");
  int counter = 0;
  for (piros_stemdet::Stem stem : message.stems) {
    ROS_INFO("  stem[%2d/%2d]: x: [%4d], y: [%4d]", ++counter,
             int(message.stems.size()), stem.x, stem.y);
  }
};
}
