/*!
  stem.cpp
  Class containing the stem detection part of the package.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#include "stem.h"

using namespace gstemdet;

Stem::Stem(const std::vector<Leaf> &leaves, const cv::Point &com) {
  if (!StemByIntersection(leaves)) {
    _stem = com;
  }
}

bool Stem::StemByIntersection(const std::vector<Leaf> &leaves) {
  int intersections = 0;
  for (unsigned int i = 0; i < leaves.size(); ++i) {  // for every leaf
    bool do_intersect = false;
    cv::Point X;
    int next_index = i + 1;

    if (i == leaves.size() - 1) {
      next_index = 0;
    }

    do_intersect = Tools::intersection(
        leaves[i].getRoot(), leaves[i].getCenter(),
        leaves[next_index].getRoot(), leaves[next_index].getCenter(), X);
    if (do_intersect == true) {
      _stem += X;
      intersections++;
    }
  }
  if (intersections > 0) {
    _stem /= intersections;
    return true;
  } else {
    return false;
  }
}

cv::Point Stem::ExtendedRoot(const cv::Point &center, const cv::Point &root) {
  double dist = Tools::distance(root, center);
  double theta = atan2(center.y - root.y, center.x - root.x);

  double mx = root.x - dist * cos(theta);
  double my = root.y - dist * sin(theta);

  return cv::Point(mx, my);
}

void Stem::drawLines(cv::Mat &img, const std::vector<Leaf> &leaves) const {
  for (unsigned int i = 0; i < leaves.size(); ++i) {
    cv::Scalar color(255 - i * 255 / leaves.size(), 0, i * 255 / leaves.size());
    cv::Point exRoot = ExtendedRoot(leaves[i].getCenter(), leaves[i].getRoot());
    cv::line(img, leaves[i].getCenter(), exRoot, color);
  }
}

void Stem::drawStem(cv::Mat &img) const {
  cv::circle(img, _stem, 3, cv::Scalar(0, 0, 0));
}

void Stem::print() const { std::cout << " Stem: " << _stem << std::endl; }
