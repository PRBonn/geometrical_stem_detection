/*!
  plant.cpp
  Class containing the structure for a single plant, contains several leaves.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#include "plant.h"

using namespace gstemdet;

Plant::Plant(const std::vector<cv::Point> &contour, const cv::Point &com) {
  _contour = contour;
  _com = com;
  convexHull();
  convexityDefects();
  extractLeaves();
  extractStem();
}

void Plant::convexHull() {
  bool clockwise = true;
  bool returnPoints = true;
  cv::convexHull(_contour, _convexHull, clockwise, returnPoints);
}

void Plant::convexityDefects() {
  cv::convexityDefects(_contour, _convexHull, _defects);
}

void Plant::extractLeaves() {
  int current;
  int first = -1;
  int prev = -1;
  for (unsigned int j = 0; j < _defects.size(); ++j) {
    float depth = _defects[j][3];
    if (depth / 256.0 > 10) {
      current = _defects[j][2];
      if (prev == -1) {
        first = current;
      } else if (prev > current) {
        std::vector<int> tempIdx(_contour.size() - prev + current + 1);
        std::iota(std::begin(tempIdx),
                  std::begin(tempIdx) + _contour.size() - prev, prev);
        std::iota(std::begin(tempIdx) + _contour.size() - prev,
                  std::end(tempIdx), 0);

        _leaves.push_back(extractLeaf(tempIdx));
      } else {
        std::vector<int> tempIdx(current - prev + 1);
        std::iota(std::begin(tempIdx), std::end(tempIdx), prev);

        _leaves.push_back(extractLeaf(tempIdx));
      }
      prev = current;
    }
  }
  if (first != -1 && first > prev) {
    std::vector<int> tempIdx(first - prev + 1);
    std::iota(std::begin(tempIdx), std::end(tempIdx), prev);

    _leaves.push_back(extractLeaf(tempIdx));
  } else if (first != -1 && first < prev) {
    std::vector<int> tempIdx(_contour.size() - prev + first + 1);
    std::iota(std::begin(tempIdx), std::begin(tempIdx) + _contour.size() - prev,
              prev);
    std::iota(std::begin(tempIdx) + _contour.size() - prev, std::end(tempIdx),
              0);

    _leaves.push_back(extractLeaf(tempIdx));
  }
}

Leaf Plant::extractLeaf(const std::vector<int> &idx) const {
  std::vector<cv::Point> tempPoints;

  for (unsigned int i = 0; i < idx.size(); ++i) {
    tempPoints.push_back(_contour[idx[i]]);
  }

  Leaf leaf(tempPoints);

  return leaf;
}

void Plant::extractStem() { _stem = Stem(_leaves, _com); }

void Plant::draw(cv::Mat &img) const {
  drawContour(img);

  // cv::Mat img_gray;
  // cvtColor( img, img_gray, CV_BGR2GRAY );

  // Test Drawing for Simple Stem
  // cv::Moments m = cv::moments(img_gray, true);
  // cv::Point simpleStem(m.m10/m.m00, m.m01/m.m00);

  drawConvexHull(img);
  drawConvexityDefects(img);
  drawCoM(img);
  for (unsigned int i = 0; i < _leaves.size(); ++i) {
    _leaves[i].draw(img);
  }
  _stem.drawLines(img, _leaves);

  // cv::circle(img, simpleStem, 3, cv::Scalar(50, 50, 50));
  _stem.drawStem(img);
}

void Plant::drawCoM(cv::Mat &img) const {
  cv::circle(img, _com, 3, cv::Scalar(50, 50, 50));
}

void Plant::drawContour(cv::Mat &img) const {
  // draw circles for each point on contour
  for (unsigned int j = 0; j < _contour.size(); j++) {  // every point
    cv::circle(img, _contour[j], 1, cv::Scalar(0, 0, 255), cv::FILLED,
               cv::LINE_AA);
  }
}

void Plant::drawConvexHull(cv::Mat &img) const {
  // draw convex hull
  cv::Point pt0 = _contour[_convexHull[_convexHull.size() - 1]];

  for (unsigned int j = 0; j < _convexHull.size(); j++) {  // every hull-point
    cv::Point pt = _contour[_convexHull[j]];
    cv::line(img, pt0, pt, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    pt0 = pt;
  }
}

void Plant::drawConvexityDefects(cv::Mat &img) const {
  /// Draw convexityDefects
  for (unsigned int j = 0; j < _defects.size(); ++j) {
    const cv::Vec4i v = _defects[j];
    float depth = v[3] / 256;
    if (depth > 10) {  //  filter defects by depth, e.g more than 10
      int defIdx = v[2];
      cv::Point ptFar(_contour[defIdx]);
      circle(img, ptFar, 4, cv::Scalar(255, 0, 0), 2);
    }
  }
}

void Plant::print() const {
  if (_leaves.size() == 1) {
    std::cout << "Plant: " << _leaves.size() << " Leaf,";
  } else {
    std::cout << "Plant: " << _leaves.size() << " Leaves,";
  }
  std::cout << " Com: [" << _com.x << ", " << _com.y << "],";
  _stem.print();
}

YAML::Node Plant::writeYAML() const {
  YAML::Node stem;
  stem["stem"]["x"] = _stem.getX();
  stem["stem"]["y"] = _stem.getY();
  return stem;
}

YAML::Node Plant::writeCoM2YAML() const {
  YAML::Node stem;
  stem["stem"]["x"] = _com.x;
  stem["stem"]["y"] = _com.y;
  return stem;
}
