/*!
  algorithm.cpp
  Class containing the leaf detection part of the package.

  @author Ferdinand Langer
  @author Leonard Mandtler
*/

#include "algorithm.h"

using namespace gstemdet;

void Algorithm::readMask(const std::string &path) {
  // load image (CV_8UC1 = only 0 and 1)
  _mask = cv::imread(path + filename_, CV_8UC1);
}

void Algorithm::readRGBMask(const std::string &path) {
  std::string name = filenameOnly(filename_);

  // load color image
  std::string file_path = path + "/" + name + "_GroundTruth_iMap.png";
#ifdef DEBUG_MODE
  std::cout << file_path << std::endl;
#endif
  cv::Mat image = cv::imread(file_path, CV_LOAD_IMAGE_GRAYSCALE);

  // find orange masked image parts
  // cv::Scalar gray = cv::Scalar(200,200,200);
  // Algorithm::blackout(image, gray);

  // cv::Scalar orange = cv::Scalar(255,100,1);
  // Algorithm::blackout(image, orange);

  // make binary image from grayscale
  cv::threshold(image, _mask, 1.0, 255.0, cv::THRESH_BINARY);
}

void Algorithm::blackout(cv::Mat &image, const cv::Scalar &color) {
  cv::Mat mask;
  cv::inRange(image, color, color, mask);
  image.setTo(cv::Scalar(0, 0, 0), mask);
}

std::string Algorithm::filenameOnly(const std::string &filename) {
  size_t lastindex = filename.find_last_of(".");
  return filename.substr(0, lastindex);
}

void Algorithm::morph(const cv::Mat &src, cv::Mat &dst, const int &m,
                      const int &element_shape, const int &operation) {
  cv::Mat element = cv::getStructuringElement(
      element_shape, cv::Size(2 * m + 1, 2 * m + 1), cv::Point(m, m));
  // Apply the morphological operations
  cv::morphologyEx(src, dst, operation, element);
}

void Algorithm::close(const int &m) {
  // MORPH_RECT, MORPH_CROSS or MORPH_ELLIPSE
  int element_shape = cv::MORPH_ELLIPSE;  // ellipse looks nicer
  int operation = cv::MORPH_CLOSE;
  morph(_mask, _mask, m, element_shape, operation);
}

void Algorithm::extractPlants(const int &mode, const int &method) {
  // 1. find contour
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(_mask, _contours, hierarchy, mode, method);

  for (unsigned int i = 0; i < _contours.size(); ++i) {
    cv::Mat just_one_contour = cv::Mat::zeros(_mask.rows, _mask.cols, CV_8U);
    drawContours(just_one_contour, _contours, i, cv::Scalar(255, 255, 255),
                 CV_FILLED, 8, hierarchy);

    // center of mass
    cv::Moments m = cv::moments(just_one_contour, true);
    cv::Point center_of_mass(m.m10 / m.m00, m.m01 / m.m00);

    Plant plant(_contours[i], center_of_mass);
    _plants.push_back(plant);
  }
}

void Algorithm::draw() {
  // blank image
  _resultIMG = cv::Mat(966, 1296, CV_8UC3);
  _resultIMG = cv::Scalar::all(0);

  drawMask(_resultIMG);

  for (unsigned int i = 0; i < _plants.size(); ++i) {
    _plants[i].draw(_resultIMG);
  }
}

void Algorithm::showResult() const {
  // show
  cv::imshow("hull", _resultIMG);
  std::cout << "Press any key!" << std::endl;
  cv::waitKey(0);
}

void Algorithm::draw(cv::Mat &img) const {
  drawMask(img);

  for (unsigned int i = 0; i < _plants.size(); ++i) {
    _plants[i].draw(img);
  }
}

void Algorithm::drawMask(cv::Mat &img) const {
  for (int i = 0; i < _mask.rows; ++i) {
    for (int j = 0; j < _mask.cols; ++j) {
      if (_mask.at<uchar>(i, j) == 255) {
        cv::Point pt(j, i);
        cv::circle(img, pt, 1, cv::Scalar(255, 255, 255), cv::FILLED);
      }
    }
  }
}

/*!
 * TODO: Hier ist der Speicherzugriffsfehler
 */
void Algorithm::printResults() const {
  std::cout << "Number of Plants: " << _contours.size() << std::endl;
  for (unsigned int i = 0; i < _plants.size(); ++i) {
    _plants[i].print();
  }
}

// Convex Hull Leaf Detection File Version
void Algorithm::LeavesWithConvexHull(const std::string &path,
                                     const std::string &mask_path, const int m,
                                     const bool drawOutput,
                                     const bool writeOutput,
                                     const bool use_com) {
  // readMask(path);
  readRGBMask(path + mask_path);
  close(m);
  extractPlants(cv::RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

#ifdef DEBUG_MODE
  printResults();
#endif
  if (drawOutput) {
    draw();
    showResult();
  }

  if (writeOutput) {
    std::string output_path;
    if (use_com) {
      output_path = path + "/CoM/" + filenameOnly(filename_) + "_stem.yaml";
    } else {
      output_path = path + "/stemdec/" + filenameOnly(filename_) + "_stem.yaml";
    }
    writeYAML(output_path, use_com);
  }
}

// Convex Hull Leaf Detection Package Version
void Algorithm::LeavesWithConvexHull(const cv::Mat &mask, const int m) {
  _contours.clear();
  _plants.clear();
  _mask = mask;
  close(m);
  extractPlants(cv::RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
}

void Algorithm::readYAML(const std::string &file_path) {
  YAML::Node yaml = YAML::LoadFile(file_path);
  filename_ = yaml["filename"].as<std::string>();
  timestamp_sec_ = yaml["time stamp sec"].as<int>();
  timestamp_nsec_ = yaml["time stamp nsec"].as<int>();
  frame_ = yaml["frame"].as<std::string>();
}

void Algorithm::writeYAML(const std::string &file_path, const bool use_com) {
  // create main node
  YAML::Node node;

  // add key & value nodes
  node["filename"] = filename_;
  node["time stamp sec"] = timestamp_sec_;
  node["time stamp nsec"] = timestamp_nsec_;
  node["frame"] = frame_;

  // nodes for each stem
  for (unsigned int i = 0; i < _plants.size(); ++i) {
    YAML::Node stem;
    if (use_com) {
      stem = _plants[i].writeCoM2YAML();
    } else {
      stem = _plants[i].writeYAML();
    }
    node["detection"].push_back(stem);
  }

  // write to file
  std::ofstream fout(file_path);
  fout << node;
#ifdef DEBUG_MODE
  std::cout << "Written yaml to " << file_path << std::endl;
#endif
}
