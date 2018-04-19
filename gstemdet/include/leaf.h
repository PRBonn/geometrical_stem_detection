/*!
  leaf.h
  Class containing the structure for a single leaf.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace gstemdet {
class Leaf {
 public:
  Leaf();

  /*!
   * Leaf constructor with contour
   * @param plant contour
   */
  explicit Leaf(const std::vector<cv::Point> &contour);

  ~Leaf() {}

  /*!
   * Calculates the root points of all leaves belonging to one mask-object
   */
  void calcRoot();

  /*!
   * Calculates the center points of all leaves belonging to one mask-object
   */
  void calcCenter();

  /*!
   * Functions to draw the leaf
   * @param image to draw below
   */
  void draw(cv::Mat &img) const;

  /*!
   * Print out Leaf to console
   */
  void print() const;

  const cv::Point &getRoot() const { return _root; }
  const cv::Point &getCenter() const { return _center; }

 private:
  /*!
   * Draw Functions
   */
  void drawContour(cv::Mat &img) const;
  void drawRoot(cv::Mat &img) const;
  void drawCenter(cv::Mat &img) const;

  //! Contour of the leaf
  std::vector<cv::Point> _contour;

  //! Root Point of the leaf
  cv::Point _root;

  //! center Point of the leaf
  cv::Point _center;
};
};
