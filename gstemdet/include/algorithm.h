/*!
  algorithm.h
  Class containing the leaf detection part of the package.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#pragma once

#include <plant.h>
#include <stem.h>
#include <iostream>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <regex>
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"

namespace gstemdet {
class Algorithm {
 public:
  /*!
   * Read vegetation mask.
   * @param image mask filepath
   */
  void readMask(const std::string &path);

  /*!
   * Read vegetation mask.
   * @param image mask filepath
   */
  void readRGBMask(const std::string &path);

  /*!
   * Replace a color in an image by black
   * @param image
   * @param color to be replaced by black
   */
  static void blackout(cv::Mat &image, const cv::Scalar &color);

  /*!
   * get just the filename to append additional string
   * @param filename with extension
   * @return filename without extension
   */
  static std::string filenameOnly(const std::string &filename);

  /*!
   * Applies morphological transformation to mask.
   * @param image mask
   * @param image mask with morphological transformation
   * @param kernel size
   * @param kernel shape
   * @param morphological transformation
   */
  static void morph(const cv::Mat &src, cv::Mat &dst, const int &m,
                    const int &element_shape, const int &operation);

  /*!
   * Applies Closing Operation to mask.
   * @param kernel size
   */
  void close(const int &m);

  /*!
   * Calculates the contours of the mask and sets CoM.
   * @param Contour retrieval mode
   * @param Contour approximation method
   */
  void extractPlants(const int &mode, const int &method);

  /*!
   * Draws the calculated convex hull and contours.
   */
  void draw();
  void draw(cv::Mat &img) const;

  void drawMask(cv::Mat &img) const;

  /*!
   * Shows the Result Image
   */
  void showResult() const;

  /*!
   * Print Results of LeavesWithConvexHull
   */
  void printResults() const;

  /*!
   * Algorithm to seperate vegetation mask to seperate leaves.
   * @param datapath
   * @param image mask filepath
   * @param kernel size
   * @param draw output boolean
   * @param center of mass boolean
   */
  void LeavesWithConvexHull(const std::string &path,
                            const std::string &mask_path, const int m,
                            const bool drawOutput = false,
                            const bool writeOutput = false,
                            const bool use_com = false);

  /*!
   * Algorithm to seperate vegetation mask to seperate leaves.
   * @param image mask
   */
  void LeavesWithConvexHull(const cv::Mat &mask, const int m);

  /*!
   * read corresponding YAML-File for 'Timestamp' and 'frame'
   * @param yaml file filepath
   */
  void readYAML(const std::string &file_path);

  /*!
   * Export as YAML file with structure as in 'test/sample_export.yaml'
   * @param yaml file filepath
   */
  void writeYAML(const std::string &file_path, const bool use_com = false);

  /*!
   * Getter
   */
  std::string getFilename() const { return filename_; };
  int getTimestampSec() const { return timestamp_sec_; };
  int getTimestampNsec() const { return timestamp_nsec_; };
  std::string getFrame() const { return frame_; };

  cv::Mat getMask() const { return _mask; };
  const cv::Mat &getResultImg() const { return _resultIMG; };
  std::vector<std::vector<cv::Point>> getContours() const { return _contours; };
  const std::vector<Plant> &getPlants() const { return _plants; }

 private:
  //! Vegetation Mask
  cv::Mat _mask;

  //! Result Image;
  cv::Mat _resultIMG;

  //! Contours
  std::vector<std::vector<cv::Point>> _contours;

  //! Plants
  std::vector<Plant> _plants;

  //! filename e.g. 'flourish-rng_2015-06-29-14-37-59_0_frame71.png'
  std::string filename_;

  //! time stamp sec e.g. '1435581549'
  int timestamp_sec_;

  //! time stamp nsec e.g. '796132968'
  int timestamp_nsec_;

  //! Frame e.g. 'jai_camera_left_optical_frame'
  std::string frame_;
};
};
