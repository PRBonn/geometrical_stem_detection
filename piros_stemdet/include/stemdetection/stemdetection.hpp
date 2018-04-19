#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "algorithm.h"
#include "piros_stemdet/Stem.h"
#include "piros_stemdet/Stems.h"
#include "tictoc.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

namespace stemdet {
class Stemdetection {
 public:
  /*!
 * Constructor.
 * @param nodeHandle the ROS node handle.
 */
  Stemdetection(ros::NodeHandle& nodeHandle);

  ~Stemdetection();

  void callbackStemDetection(const sensor_msgs::ImageConstPtr& maskImage,
                             const sensor_msgs::ImageConstPtr& rgbImage,
                             const sensor_msgs::ImageConstPtr& nirImage);

  void publishStems(const sensor_msgs::ImageConstPtr& maskImage) const;
  void publishResultImage(const sensor_msgs::ImageConstPtr& maskImage);

 private:
  /*!
* Reads and verifies the ROS parameters.
* @return true if successful.
*/
  bool readParameters();

  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;

  //! ROS topic names to subscribe to
  std::string maskImageTopic_;
  std::string rgbImageTopic_;
  std::string nirImageTopic_;

  //! ROS topics name to publish to
  std::string imagePublisherTopic_;
  std::string stemPublisherTopic_;

  //! ROS topic subscriber
  image_transport::SubscriberFilter _maskImageSubscriber;
  image_transport::SubscriberFilter _rgbImageSubscriber;
  image_transport::SubscriberFilter _nirImageSubscriber;

  //! ROS topic publisher.
  image_transport::Publisher imagePublisher_;
  ros::Publisher stemPublisher_;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>
      syncPolicy_;
  message_filters::Synchronizer<syncPolicy_>* synchronizer_;

  int closingSize_;

  int verbose_;

  gstemdet::Algorithm algorithm_;
};
};
