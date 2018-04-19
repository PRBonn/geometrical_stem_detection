#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace vegsec {

/**
 * @brief      Class for calculating indexes and masks.
 */
class VegSec {
 public:
  /**
   * @brief      Constructor
   *
   * @param      gsd            The ground sampling distance [mm/px]
   * @param      min_area_blob  The minimum area of a desired detected blob
   */
  VegSec(double &gsd, double &min_area_blob);

  /**
   * @brief      Destroys the object.
   */
  virtual ~VegSec();

  /**
   * @brief      Calculates the Excess Green Index
   *
   * @param[in]  rgb   The rgb image to use
   *
   * @return     The exgreen index
   */
  cv::Mat exgr(const cv::Mat &rgb);

  /**
   * @brief      Calculates the NDVI Index
   *
   * @param[in]  rgb   The rgb image to use
   * @param[in]  nir   The nir image to use
   *
   * @return     The ndvi index
   */
  cv::Mat ndvi(const cv::Mat &rgb, const cv::Mat &nir);

  /**
   * @brief      Calculates the mas from an index
   *
   * @param[in]  idx       The index to use
   * @param      gb_k      The gaussian blur kernel size
   * @param      morph_k   The morphological op kernel size
   * @param[in]  see_smal  See small blobs too (in gray)
   *
   * @return     The thresholded mask
   */
  cv::Mat mask(const cv::Mat &idx, int &gb_k, int &morph_k,
               bool see_small = false);

 private:
  double gsd_;            // ground sampling distance
  double min_area_blob_;  // min area to consider vegetation
};

}  // namespace vegsec
