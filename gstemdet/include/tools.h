/*!
  tools.h
  Class containing various useful functions.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>

namespace gstemdet {
class Tools {
 public:
  Tools();
  ~Tools();

  /*!
   * calculates the distance between two points
   * @param first point
   * @param second point
   */
  static double distance(const cv::Point_<int> &first,
                         const cv::Point_<int> &second);

  /*!
   * Finds the intersection of two lines, or returns false.
   * @param first point of line P
   * @param second point of line P
   * @param first point of line Q
   * @param second point of line Q
   * @param intersection point
   * @return true if intersection found
   */
  static bool intersection(const cv::Point &P1, const cv::Point &P2,
                           const cv::Point &Q1, const cv::Point &Q2,
                           cv::Point &r);
};
};
