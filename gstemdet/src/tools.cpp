/*!
  tools.cpp
  Class containing various useful functions.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#include "tools.h"

using namespace gstemdet;

double Tools::distance(const cv::Point_<int> &first,
                       const cv::Point_<int> &second) {
  return std::sqrt(std::pow(first.x - second.x, 2) +
                   std::pow(first.y - second.y, 2));
}

bool Tools::intersection(const cv::Point &P1, const cv::Point &P2,
                         const cv::Point &Q1, const cv::Point &Q2,
                         cv::Point &r) {
  // The lines are defined by (P1, P2) and (Q1, Q2).
  cv::Point x = Q1 - P1;
  cv::Point dP = P2 - P1;
  cv::Point dQ = Q2 - Q1;

  float cross = dP.x * dQ.y - dP.y * dQ.x;
  // smaller than some epsilon
  if (std::abs(cross) < 1e-8) {
    return false;
  }

  double t1 = (x.x * dQ.y - x.y * dQ.x) / cross;
  r = P1 + dP * t1;
  return true;
}
