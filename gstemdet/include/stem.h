/*!
  stem.h
  Class containing the stem detection part of the package.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#pragma once

#include <leaf.h>
#include <tools.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <vector>

namespace gstemdet {
class Stem {
 public:
  Stem() {}

  /*!
   * Initializes the stem according to the leaves of the plant, defaults to com
   */
  Stem(const std::vector<Leaf> &leaves, const cv::Point &com);

  ~Stem() {}

  /*!
   * calculates the stem by intersecting lines
   */
  bool StemByIntersection(const std::vector<Leaf> &leaves);

  /*!
   * Draw all lines between leaf-center and leaf-root
   * @param image to draw on
   */
  void drawLines(cv::Mat &img, const std::vector<Leaf> &leaves) const;

  /*!
   * Draw the stem
   * @param image to draw on
   */
  void drawStem(cv::Mat &img) const;

  /*!
   * Print out Stem to console
   */
  void print() const;

  /*!
   * Getter
   */
  int getX() const { return _stem.x; }
  int getY() const { return _stem.y; }

 private:
  //! Stem coordinate
  cv::Point _stem;

  /*!
   * Extendet leaf-root for drawing a line longer than leaf-root to leaf-center
   * @param leaf-root
   * @param leaf-center
   * @return mean of leaf
   */
  static cv::Point ExtendedRoot(const cv::Point &center, const cv::Point &root);
};
};
