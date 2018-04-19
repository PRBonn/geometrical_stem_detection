/*!
  leaf.cpp
  Class containing the structure for a single leaf.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#include "leaf.h"

using namespace gstemdet;

Leaf::Leaf(const std::vector<cv::Point> &contour) {
  _contour = contour;
  calcRoot();
  calcCenter();
}

void Leaf::calcRoot() {
  cv::Point cuttingFirst = _contour[0];
  cv::Point cuttingSecond = _contour[_contour.size() - 1];

  _root = cv::Point((cuttingFirst.x + cuttingSecond.x) / 2,
                    (cuttingFirst.y + cuttingSecond.y) / 2);
}

void Leaf::calcCenter() {
  // calculate the center by getting the mean of all contour points of a leaf
  _center = cv::Point(0, 0);

  for (unsigned int k = 0; k < _contour.size(); ++k) {
    cv::Point currentPoint = _contour[k];
    _center += currentPoint;
  }

  _center.x /= _contour.size();
  _center.y /= _contour.size();
}

void Leaf::draw(cv::Mat &img) const {
  drawContour(img);
  drawRoot(img);
  drawCenter(img);
}

void Leaf::drawContour(cv::Mat &img) const {
  cv::Scalar color(100, 0, 255);
  for (unsigned int k = 0; k < _contour.size(); ++k) {
    circle(img, _contour[k], 1, color);
  }
}

void Leaf::drawRoot(cv::Mat &img) const {
  cv::Scalar color(0, 255, 0);
  circle(img, _root, 3, color);
}

void Leaf::drawCenter(cv::Mat &img) const {
  cv::Scalar color(255, 0, 0);
  circle(img, _center, 3, color);
}

void Leaf::print() const {
  std::cout << "  Leaf: " << _center << ", " << _root << std::endl;
  std::cout << "    Kontur: " << std::endl;
  for (unsigned int i = 0; i < _contour.size(); ++i) {
    if (i == 0) {
      std::cout << "     [" << _contour[i];

    } else if (i % 10 == 0) {
      std::cout << "," << std::endl << "     [" << _contour[i];
    } else {
      std::cout << ", " << _contour[i];
    }
  }
  std::cout << "]" << std::endl;
}
