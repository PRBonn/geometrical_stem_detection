#include "vegsec.h"

namespace vegsec {

VegSec::VegSec(double &gsd, double &min_area_blob)
    : gsd_(gsd), min_area_blob_(min_area_blob) {}

VegSec::~VegSec() {}

cv::Mat VegSec::exgr(const cv::Mat &rgb) {
  // compute Excess Green index
  cv::Mat ExG(rgb.rows, rgb.cols, CV_32FC1, cv::Scalar(0.0));

  // loop and calculate index
  for (int y = 0; y < rgb.rows; ++y) {
    // get row pointers (fast opencv impl)
    const cv::Vec3b *row_bgr = rgb.ptr<cv::Vec3b>(y);
    float *row_exgr = ExG.ptr<float>(y);

    for (int x = 0; x < rgb.cols; ++x) {
      float R, G, B, SUM;
      R = static_cast<float>(row_bgr[x][2]) / 255.0;
      G = static_cast<float>(row_bgr[x][1]) / 255.0;
      B = static_cast<float>(row_bgr[x][0]) / 255.0;
      SUM = R + G + B;

      // epsilon
      SUM = R + G + B;
      if (SUM == 0.0) SUM = 0.01;

      // norm
      R = R / SUM;
      G = G / SUM;
      B = B / SUM;

      row_exgr[x] = 2.0 * G - R - B;
    }
  }

  // normalize to 0..255
  cv::normalize(ExG, ExG, 0, 255, cv::NORM_MINMAX);

  // convert to uint8_t
  ExG.convertTo(ExG, CV_8UC1);

  // return only the green
  return ExG;
}

cv::Mat VegSec::ndvi(const cv::Mat &rgb, const cv::Mat &nir) {
  // compute Excess Green index
  cv::Mat NDVI(rgb.rows, rgb.cols, CV_32FC1, cv::Scalar(0.0));

  // loop and calculate index
  for (int y = 0; y < rgb.rows; ++y) {
    // get row pointers (fast opencv impl)
    const cv::Vec3b *row_bgr = rgb.ptr<cv::Vec3b>(y);
    const uint8_t *row_nir = nir.ptr<uint8_t>(y);
    float *row_ndvi = NDVI.ptr<float>(y);
    for (int x = 0; x < rgb.cols; ++x) {
      float R, N, DEN;
      R = static_cast<float>(row_bgr[x][2]);
      N = static_cast<float>(row_nir[x]);

      // epsilon
      DEN = R + N;
      if (DEN == 0.0) {
        DEN = 0.01;
      }

      row_ndvi[x] = (N - R) / DEN;
    }
  }

  // normalize to 0..255
  cv::normalize(NDVI, NDVI, 0, 255, cv::NORM_MINMAX);

  // convert to uint8_t
  NDVI.convertTo(NDVI, CV_8UC1);

  // return only the green
  return NDVI;
}

cv::Mat VegSec::mask(const cv::Mat &idx, int &gb_k, int &morph_k,
                     bool see_small) {
  // Create placeholder for the mask
  cv::Mat mask;

  // Get a local idx
  idx.convertTo(mask, CV_8UC1);

  // Smooth
  cv::GaussianBlur(mask, mask, cv::Size(gb_k, gb_k), 0, 0);

  // Automatic threshold
  cv::threshold(mask, mask, 0, 255, cv::THRESH_TRIANGLE);

  // Openings and closings to reduce noise and gaps
  cv::Mat se =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_k, morph_k));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, se);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, se);

  // Detect blobs of minimum size
  float min_px = min_area_blob_ / (gsd_ * gsd_);
  // std::cout << "min_area_blob_ = " << min_area_blob_ << std::endl
  //           << "gsd_ = " << gsd_ << std::endl
  //           << "min_px = " << min_px << std::endl;

  // Get connected componets
  cv::Mat labels, stats, centroids;
  mask.convertTo(mask, CV_8UC1);
  int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);

  // generate labels
  std::vector<uint8_t> colors(n);
  colors[0] = 0;  // background pixels remain black.
  for (int i = 1; i < n; i++) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) < min_px) {
      if (see_small) {
        colors[i] = 60;  // small regions are palid gray. Just debug!
      } else {
        colors[i] = 0;  // small regions are black
      }
    } else {
      colors[i] = 255;  // normal regions are white
    }
  }

  // paint the mask
  for (int y = 0; y < mask.rows; y++) {
    int *row_labels = labels.ptr<int>(y);
    uint8_t *row_mask = mask.ptr<uint8_t>(y);
    for (int x = 0; x < mask.cols; x++) {
      row_mask[x] = colors[row_labels[x]];
    }
  }

  return mask;
}

};  // namespace vegsec
