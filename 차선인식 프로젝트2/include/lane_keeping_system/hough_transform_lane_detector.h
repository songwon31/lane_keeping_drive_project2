/**
 * @file hough_transform_lane_detector.h
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief Hough Transform Lane Detector class header file
 * @version 0.2
 * @date 2022-11-27
 */
#ifndef HOUGH_TRANSFORM_LANE_DETECTOR_H_
#define HOUGH_TRANSFORM_LANE_DETECTOR_H_
#include <yaml-cpp/yaml.h>

#include <iostream>

#include "opencv2/opencv.hpp"

namespace xycar {
class HoughTransformLaneDetector final {
public:
  // Construct a new Hough Transform Lane Detector object
  HoughTransformLaneDetector(const YAML::Node &config);
  // Detect Lane position
  std::pair<int, int> getLanePosition(const cv::Mat &image);
  // Draw rectagles on debug image
  void draw_rectangles(int lpos, int rpos, int ma_mpos);

  // Get debug image
  cv::Mat *getDebugFrame();

private:
  // Set parameters from config file
  void set(const YAML::Node &config);
  // Divide lines into left and right
  std::pair<std::vector<int>, std::vector<int>> divideLines(
    const std::vector<cv::Vec4i> &lines);
  // get position of left and right line
  int get_line_pos(const std::vector<cv::Vec4i> &lines,
                   const std::vector<int> &line_index,
                   bool direction);
  // get slpoe and intercept of line
  std::pair<float, float> get_line_params(const std::vector<cv::Vec4i> &lines,
                                          const std::vector<int> &line_index);
  // draw line on debug image
  void draw_lines(const std::vector<cv::Vec4i> &lines,
                  const std::vector<int> &left_line_index,
                  const std::vector<int> &right_line_index);

private:
  // Hough Transform Parameters
  enum kHoughIndex { x1 = 0, y1, x2, y2 };
  static const double kHoughRho;
  static const double kHoughTheta;
  static const int kDebgLineWidth = 2;
  static const int kDebugRectangleHalfWidth = 5;
  static const int kDebugRectangleStartHeight = 15;
  static const int kDebugRectangleEndHeight = 25;
  int canny_edge_low_threshold_;
  int canny_edge_high_threshold_;
  float hough_line_slope_range_;
  int hough_threshold_;
  int hough_min_line_length_;
  int hough_max_line_gap_;

  // Image parameters
  int image_width_;
  int image_height_;
  int roi_start_height_;
  int roi_height_;

  // line type falg
  static const bool kLeftLane = true;
  static const bool kRightLane = false;

  // Debug Image and flag
  cv::Mat debug_frame_;
  bool debug_;
};
}  // namespace xycar
#endif  // HOUGH_TRANSFORM_LANE_DETECTOR_H_
