#include "vegsec.h"
#include <chrono>
#include "vegsec_handler.h"

namespace vegsec {

VegSecHandler::VegSecHandler(ros::NodeHandle& node_handle)
    : node_handle_(node_handle), it_(node_handle) {
  //! Try to read config file,
  if (!ReadParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // spawn image transport

  // Subscribe to images to infer
  rgb_subs_.subscribe(it_, rgb_sub_topic_, 1);
  nir_subs_.subscribe(it_, nir_sub_topic_, 1);

  // sync policy
  sync_ =
      std::unique_ptr<sync_t>(new sync_t(sync_pol_t(10), rgb_subs_, nir_subs_));
  sync_->registerCallback(
      boost::bind(&VegSecHandler::rgb_nir_callback, this, _1, _2));

  // Advertise our topics
  exgr_publ_ = it_.advertise(exgr_pub_topic_, 1);
  ndvi_publ_ = it_.advertise(ndvi_pub_topic_, 1);
  mask_publ_ = it_.advertise(mask_pub_topic_, 1);

  // create core object for segmentation and indexes
  veg_core_ = std::unique_ptr<VegSec>(new VegSec(gsd_, min_area_blob_));

  ROS_INFO("Successfully launched node.");
}

VegSecHandler::~VegSecHandler() {
  // No need to do anything
}

bool VegSecHandler::ReadParameters() {
  std::string mask_index;
  if (!node_handle_.getParam("in_rgb_topic", rgb_sub_topic_) ||
      !node_handle_.getParam("in_nir_topic", nir_sub_topic_) ||
      !node_handle_.getParam("out_exgr", exgr_pub_topic_) ||
      !node_handle_.getParam("out_ndvi", ndvi_pub_topic_) ||
      !node_handle_.getParam("out_mask", mask_pub_topic_) ||
      !node_handle_.getParam("mask_index", mask_index) ||
      !node_handle_.getParam("gb_k_size", gb_k_size_) ||
      !node_handle_.getParam("morph_k_size", morph_k_size_) ||
      !node_handle_.getParam("gsd", gsd_) ||
      !node_handle_.getParam("min_area_blob", min_area_blob_) ||
      !node_handle_.getParam("plot_small_obj", plot_small_obj_))
    return false;

  // get the actual enum index to use
  if (mask_index == "ndvi")
    index_ = INDEX_NDVI;
  else if (mask_index == "exgr") {
    index_ = INDEX_EXGR;
  } else {
    return false;
  }

  return true;
}

int VegSecHandler::to_bgr(const sm::ImageConstPtr& msg, cv::Mat& output) {
  // Get the image
  cv_bridge::CvImageConstPtr cv_img;
  cv_img = cv_bridge::toCvShare(msg);

  // change to bgr according to encoding
  if (msg->encoding == "bayer_rggb8") {
    ROS_INFO("Converting BAYER_RGGB8 to BGR");
    cv::cvtColor(cv_img->image, output, cv::COLOR_BayerBG2BGR);
  } else if (msg->encoding == "bgr8") {
    ROS_INFO("Converting BGR8 to BGR");
    output = cv_img->image;
  } else if (msg->encoding == "rgb8") {
    ROS_INFO("Converting RGB8 to BGR");
    cv::cvtColor(cv_img->image, output, cv::COLOR_RGB2BGR);
  } else {
    ROS_ERROR("Colorspace conversion non implemented. Skip...");
    return 1;
  }

  // convert to bgr
  output.convertTo(output, CV_8UC3);

  return 0;
}

int VegSecHandler::to_mono8(const sm::ImageConstPtr& msg, cv::Mat& output) {
  // Get the image
  cv_bridge::CvImageConstPtr cv_img;
  cv_img = cv_bridge::toCvShare(msg);

  // change to bgr according to encoding
  if (msg->encoding == "mono8") {
    ROS_INFO("Converting mono8 to mono8");
    output = cv_img->image;
  } else {
    ROS_ERROR("Colorspace conversion non implemented. Skip...");
    return 1;
  }

  // convert to bgr
  output.convertTo(output, CV_8UC1);

  return 0;
}

void VegSecHandler::rgb_nir_callback(const sm::ImageConstPtr& rgb_msg,
                                     const sm::ImageConstPtr& nir_msg) {
  ROS_INFO("Image received.");

  // elapsed_inference time
  auto start_time = std::chrono::high_resolution_clock::now();

  // to bgr
  cv::Mat cv_img_bgr, cv_img_nir;
  if (to_bgr(rgb_msg, cv_img_bgr)) {
    ROS_ERROR("Problem converting to BGR");
    return;
  }

  // to mono8
  if (to_mono8(nir_msg, cv_img_nir)) {
    ROS_ERROR("Problem converting to Mono");
    return;
  }

  // segment using core
  cv::Mat exgr = veg_core_->exgr(cv_img_bgr);
  cv::Mat ndvi = veg_core_->ndvi(cv_img_bgr, cv_img_nir);

  cv::Mat mask;
  switch (index_) {
    case INDEX_EXGR:
      mask = veg_core_->mask(exgr, gb_k_size_, morph_k_size_, plot_small_obj_);
      break;
    case INDEX_NDVI:
      mask = veg_core_->mask(ndvi, gb_k_size_, morph_k_size_, plot_small_obj_);
      break;
    default:
      ROS_ERROR("USING UNMAPPED INDEX");
  }

  // Send the exgr
  sensor_msgs::ImagePtr exgr_msg =
      cv_bridge::CvImage(rgb_msg->header, "mono8", exgr).toImageMsg();
  exgr_publ_.publish(exgr_msg);

  // Send the ndvi
  sensor_msgs::ImagePtr ndvi_msg =
      cv_bridge::CvImage(rgb_msg->header, "mono8", ndvi).toImageMsg();
  ndvi_publ_.publish(ndvi_msg);

  // Send the mask
  sensor_msgs::ImagePtr mask_msg =
      cv_bridge::CvImage(rgb_msg->header, "mono8", mask).toImageMsg();
  mask_publ_.publish(mask_msg);

  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::high_resolution_clock::now() - start_time)
                     .count();

  ROS_INFO("Time to calculate: %ldms", elapsed);
}

}  // namespace vegsec
