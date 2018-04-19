#include "stemdetection/stemdetection.hpp"

using namespace gstemdet;
namespace stemdet {
Stemdetection::Stemdetection(ros::NodeHandle &nodeHandle)
    : nh_(nodeHandle), it_(nh_) {
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  _maskImageSubscriber.subscribe(it_, maskImageTopic_, 1);
  _rgbImageSubscriber.subscribe(it_, rgbImageTopic_, 1);
  _nirImageSubscriber.subscribe(it_, nirImageTopic_, 1);

  synchronizer_ = new message_filters::Synchronizer<syncPolicy_>(
      syncPolicy_(200), _maskImageSubscriber, _rgbImageSubscriber,
      _nirImageSubscriber);

  synchronizer_->registerCallback(
      boost::bind(&Stemdetection::callbackStemDetection, this, _1, _2, _3));

  if (verbose_ == 2) imagePublisher_ = it_.advertise(imagePublisherTopic_, 1);
  stemPublisher_ =
      nh_.advertise<piros_stemdet::Stems>(stemPublisherTopic_, 1);
}

Stemdetection::~Stemdetection() {}

bool Stemdetection::readParameters() {
  if (!nh_.getParam("in_mask", maskImageTopic_) ||
      !nh_.getParam("in_rgb", rgbImageTopic_) ||
      !nh_.getParam("in_nir", nirImageTopic_) ||
      !nh_.getParam("out_mask", imagePublisherTopic_) ||
      !nh_.getParam("out_stem", stemPublisherTopic_) ||
      !nh_.getParam("closing_size", closingSize_) ||
      !nh_.getParam("verbose", verbose_))
    return false;
  return true;
}

void Stemdetection::callbackStemDetection(
    const sensor_msgs::ImageConstPtr &maskImage,
    const sensor_msgs::ImageConstPtr &rgbImage,
    const sensor_msgs::ImageConstPtr &nirImage) {
  cv_bridge::CvImagePtr cv_ptr;
  TicToc t;

  try {
    cv_ptr =
        cv_bridge::toCvCopy(maskImage, sensor_msgs::image_encodings::MONO8);
    if (verbose_ > 0) ROS_INFO("Stem Detection: Received Mask");

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat mask = cv_ptr->image;

  // Timing
  if (verbose_ > 0) {
    t.tic();
  }

  //! call detection algorithm
  algorithm_.LeavesWithConvexHull(mask, closingSize_);

  if (verbose_ > 0) {
    double time = t.toc();  // seconds
    ROS_INFO("Stem Detection: Calculated Stems in %1.0fms", time * 1000);
  }

  //! publish stems
  publishStems(maskImage);

  //! publish Debug Images if Debug-Mode is on
  if (verbose_ == 2) {
    publishResultImage(maskImage);
  }
}

void Stemdetection::publishStems(
    const sensor_msgs::ImageConstPtr &maskImage) const {
  piros_stemdet::Stems stemlist;

  std::vector<Plant> plants = algorithm_.getPlants();

  for (auto plant : plants) {
    piros_stemdet::Stem single_stem;

    single_stem.x = plant.getStem().getX();
    single_stem.y = plant.getStem().getY();

    stemlist.stems.push_back(single_stem);
  }

  stemlist.header = maskImage->header;

  stemPublisher_.publish(stemlist);
  if (verbose_ > 0) ROS_INFO("Stem Detection: Published Stems");
}

void Stemdetection::publishResultImage(
    const sensor_msgs::ImageConstPtr &maskImage) {
  algorithm_.draw();
  cv::Mat resultImg = algorithm_.getResultImg();

  sensor_msgs::ImagePtr resultImgMsg =
      cv_bridge::CvImage(maskImage->header, "bgr8", resultImg).toImageMsg();

  imagePublisher_.publish(resultImgMsg);
  if (verbose_ > 0) ROS_INFO("Stem Detection: Published Debug Image");
}
}
