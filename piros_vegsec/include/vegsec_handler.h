#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <string>
#include "vegsec.h"

namespace mf = message_filters;
namespace it = image_transport;
namespace sm = sensor_msgs;

typedef mf::sync_policies::ApproximateTime<sm::Image, sm::Image> sync_pol_t;
typedef mf::Synchronizer<sync_pol_t> sync_t;

namespace vegsec {

/**
 * @brief      Main class for the node to handle the Veg Segmentation.
 */
class VegSecHandler {
 public:
  /**
   * @brief      Constructor.
   *
   * @param      node_handle  The node handle
   */
  VegSecHandler(ros::NodeHandle& node_handle);

  /**
   * @brief      Destroys the object.
   */
  virtual ~VegSecHandler();

 private:
  /**
   * @brief      Reads and verifies the ROS parameters.
   *
   * @return     true if successful.
   */
  bool ReadParameters();

  /**
   * @brief      Convert ROS message to cv mat.
   *
   * @param[in]  msg     The received message
   * @param      output  The output cv mat
   *
   * @return     retcode (0 success, -1 fail)
   */
  int to_bgr(const sm::ImageConstPtr& msg, cv::Mat& output);

  /**
   * @brief      Convert ROS message to cv mat
   *
   * @param[in]  msg     The received message
   * @param      output  The output cv mat
   *
   * @return     retcode (0 success, -1 fail)
   */
  int to_mono8(const sm::ImageConstPtr& msg, cv::Mat& output);

  /**
     * @brief      ROS topic callback method.
     *
     * @param[in]  msg   The received message
     */
  void rgb_nir_callback(const sm::ImageConstPtr& rgb_msg,
                        const sm::ImageConstPtr& nir_msg);

  //! ROS stuff for node handling
  ros::NodeHandle node_handle_;
  std::string rgb_sub_topic_;   // name of rgb topic to subscribe to
  std::string nir_sub_topic_;   // name of nir topic to subscribe to
  std::string exgr_pub_topic_;  // name of exgreen topic to publish to
  std::string ndvi_pub_topic_;  // name of ndvi topic to publish to
  std::string mask_pub_topic_;  // name of mask topic to publish to
  bool plot_small_obj_;         // debug small objects making them gray in mask

  //! plant parameters
  int gb_k_size_;
  int morph_k_size_;
  double gsd_;
  double min_area_blob_;

  //! ROS topic subscribers and publishers.
  it::ImageTransport it_;
  std::unique_ptr<sync_t> sync_;
  it::SubscriberFilter rgb_subs_;
  it::SubscriberFilter nir_subs_;
  it::Publisher exgr_publ_;
  it::Publisher ndvi_publ_;
  it::Publisher mask_publ_;

  std::unique_ptr<VegSec> veg_core_;  // core object to run algorithms

  // index to use
  enum { INDEX_NDVI = 0, INDEX_EXGR = 1 } index_;
};

}  // namespace vegsec
