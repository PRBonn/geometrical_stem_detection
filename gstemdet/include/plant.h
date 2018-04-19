/*!
  plant.h
  Class containing the structure for a single plant, contains several leaves.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#pragma once

#include <leaf.h>
#include <stem.h>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <vector>

namespace gstemdet {
class Plant {
 public:
  Plant() {}

  /*!
   * Plant constructor with contour and com
   * @param plant contour
   * @param center of mass
   */
  Plant(const std::vector<cv::Point> &contour, const cv::Point &com);

  ~Plant() {}

  /*!
   * Functions to draw the plant
   * @param image to draw below
   */
  void draw(cv::Mat &img) const;

  /*!
   * Print out plant to console
   */
  void print() const;

  /*!
   * Returns the Stem of this plant as a YAML-Node
   */
  YAML::Node writeYAML() const;

  /*!
   * Returns the CoM of this plant as a YAML-Node
   */
  YAML::Node writeCoM2YAML() const;

  /*!
   * Getter
   */
  Stem getStem() const { return _stem; };

 private:
  /*!
   * Calculates the convex hull of the plant
   */
  void convexHull();

  /*!
   * Calculates the convexity defects of all convex hulls.
   */
  void convexityDefects();

  /*!
   * Extracts the contour of leaves belonging to one mask-object
   */
  void extractLeaves();

  /*!
   * Extracts the Contour of a Leaf using the contour of the plant and a
   * corresponding id-vector
   * @param corresponding id-vector
   * @return extracted leaf
   */
  Leaf extractLeaf(const std::vector<int> &idx) const;

  /*!
   * Extracts the Stem of the plant according to the leaves
   */
  void extractStem();

  /*!
   * Draw Functions
   */
  void drawCoM(cv::Mat &img) const;
  void drawContour(cv::Mat &img) const;
  void drawConvexHull(cv::Mat &img) const;
  void drawConvexityDefects(cv::Mat &img) const;

  //! Contour of one plant
  std::vector<cv::Point> _contour;

  //! convex Hull of the plant
  std::vector<int> _convexHull;

  //! convexity defects
  std::vector<cv::Vec4i> _defects;

  //! Leaves belonging to this plant;
  std::vector<Leaf> _leaves;

  //! Stem belonging to this plant;
  Stem _stem;

  //! Center of Mass
  cv::Point _com;
};
};
