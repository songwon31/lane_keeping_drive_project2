/**
 * @file hough_transform_lane_detector.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief hough transform lane detector class source file
 * @version 0.2
 * @date 2022-11-27
 */
#include "lane_keeping_system/hough_transform_lane_detector.h"

namespace xycar {
const double HoughTransformLaneDetector::kHoughRho = 4.0;
const double HoughTransformLaneDetector::kHoughTheta = CV_PI / 180.0;

HoughTransformLaneDetector::HoughTransformLaneDetector(
  const YAML::Node &config) {
  set(config);
}

void HoughTransformLaneDetector::set(const YAML::Node &config) {
  image_width_ = config["IMAGE"]["WIDTH"].as<int>();
  image_height_ = config["IMAGE"]["HEIGHT"].as<int>();
  roi_start_height_ = config["IMAGE"]["ROI_START_HEIGHT"].as<int>();
  roi_height_ = config["IMAGE"]["ROI_HEIGHT"].as<int>();
  canny_edge_low_threshold_ = config["CANNY"]["LOW_THRESHOLD"].as<int>();
  canny_edge_high_threshold_ = config["CANNY"]["HIGH_THRESHOLD"].as<int>();
  hough_line_slope_range_ = config["HOUGH"]["ABS_SLOPE_RANGE"].as<float>();
  hough_threshold_ = config["HOUGH"]["THRESHOLD"].as<int>();
  hough_min_line_length_ = config["HOUGH"]["MIN_LINE_LENGTH"].as<int>();
  hough_max_line_gap_ = config["HOUGH"]["MAX_LINE_GAP"].as<int>();
  debug_ = config["DEBUG"].as<bool>();
}

cv::Mat *HoughTransformLaneDetector::getDebugFrame() { return &debug_frame_; }

std::pair<float, float> HoughTransformLaneDetector::get_line_params(
  const std::vector<cv::Vec4i> &lines, const std::vector<int> &line_index) {
  int lines_size = line_index.size();
  if (lines_size == 0) {
    return std::pair<float, float>(0.0f, 0.0f);
  }

  int x1, y1, x2, y2;
  float x_sum = 0.0f, y_sum = 0.0f, m_sum = 0.0f;
  for (int i = 0; i < lines_size; ++i) {
    x1 = lines[line_index[i]][kHoughIndex::x1],
    y1 = lines[line_index[i]][kHoughIndex::y1];
    x2 = lines[line_index[i]][kHoughIndex::x2],
    y2 = lines[line_index[i]][kHoughIndex::y2];

    x_sum += x1 + x2;
    y_sum += y1 + y2;
    m_sum += (float)(y2 - y1) / (x2 - x1);
  }

  float x_avg, y_avg, m, b;
  x_avg = x_sum / (lines_size * 2);
  y_avg = y_sum / (lines_size * 2);
  m = m_sum / lines_size;
  b = y_avg - m * x_avg;

  std::pair<float, float> m_and_b(m, b);
  return m_and_b;
}

int HoughTransformLaneDetector::get_line_pos(
  const std::vector<cv::Vec4i> &lines,
  const std::vector<int> &line_index,
  const bool direction) {
  float m, b;
  std::tie(m, b) = get_line_params(lines, line_index);

  float y, pos;
  if (m == 0.0 && b == 0.0) {
    if (direction == kLeftLane) {
      pos = 0.0f;
    } else {
      pos = (float)image_width_;
    }
  } else {
    y = (float)roi_height_ * 0.5;
    pos = (y - b) / m;
  }
  return std::round(pos);
}

std::pair<std::vector<int>, std::vector<int>>
HoughTransformLaneDetector::divideLines(const std::vector<cv::Vec4i> &lines) {
  int lines_size = lines.size();
  std::vector<int> left_line_index;
  std::vector<int> right_line_index;
  left_line_index.reserve(lines_size);
  right_line_index.reserve(lines_size);
  int x1, y1, x2, y2;
  float slope;
  float left_line_x_sum = 0.0f;
  float right_line_x_sum = 0.0f;
  float left_x_avg, right_x_avg;

  for (int i = 0; i < lines_size; ++i) {
    x1 = lines[i][kHoughIndex::x1], y1 = lines[i][kHoughIndex::y1];
    x2 = lines[i][kHoughIndex::x2], y2 = lines[i][kHoughIndex::y2];
    if (x2 - x1 == 0) {
      slope = 0.0f;
    } else {
      slope = (float)(y2 - y1) / (x2 - x1);
    }
    if (-hough_line_slope_range_ <= slope && slope < 0) {
      left_line_x_sum += (float)(x1 + x2) * 0.5;
      left_line_index.push_back(i);
    } else if (0 < slope && slope <= hough_line_slope_range_) {
      right_line_x_sum += (float)(x1 + x2) * 0.5;
      right_line_index.push_back(i);
    }
  }
  int left_lines_size = left_line_index.size();
  int right_lines_size = right_line_index.size();
  if (left_lines_size != 0 && right_lines_size != 0) {
    left_x_avg = left_line_x_sum / left_lines_size;
    right_x_avg = right_line_x_sum / right_lines_size;
    if (left_x_avg > right_x_avg) {
      left_line_index.clear();
      right_line_index.clear();
      std::cout << "------Invalid Path!------\n";
    }
  }

  return std::pair<std::vector<int>, std::vector<int>>(
    std::move(left_line_index), std::move(right_line_index));
}

std::pair<int, int> HoughTransformLaneDetector::getLanePosition(
  const cv::Mat &image) {
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  cv::Mat canny_image;
  cv::Canny(gray_image,
            canny_image,
            canny_edge_low_threshold_,
            canny_edge_high_threshold_);
  cv::Mat roi =
    canny_image(cv::Rect(0, roi_start_height_, image_width_, roi_height_));
  std::vector<cv::Vec4i> all_lines;
  cv::HoughLinesP(roi,
                  all_lines,
                  kHoughRho,
                  kHoughTheta,
                  hough_threshold_,
                  hough_min_line_length_,
                  hough_max_line_gap_);
  if (all_lines.size() == 0) {
    return std::pair<int, int>(0, 0);
  }

  std::vector<int> left_line_index, right_line_index;
  std::tie(left_line_index, right_line_index) =
    std::move(divideLines(all_lines));

  int lpos = get_line_pos(all_lines, left_line_index, kLeftLane);
  int rpos = get_line_pos(all_lines, right_line_index, kRightLane);

  if (debug_) {
    image.copyTo(debug_frame_);
    draw_lines(all_lines, left_line_index, right_line_index);
  }

  return std::pair<int, int>(lpos, rpos);
}

void HoughTransformLaneDetector::draw_lines(
  const std::vector<cv::Vec4i> &lines,
  const std::vector<int> &left_line_index,
  const std::vector<int> &right_line_index) {
  cv::Point2i pt1, pt2;
  cv::Scalar color;
  for (int i = 0; i < left_line_index.size(); ++i) {
    pt1 = cv::Point2i(
      lines[left_line_index[i]][kHoughIndex::x1],
      lines[left_line_index[i]][kHoughIndex::y1] + roi_start_height_);
    pt2 = cv::Point2i(
      lines[left_line_index[i]][kHoughIndex::x2],
      lines[left_line_index[i]][kHoughIndex::y2] + roi_start_height_);
    int r, g, b;
    r = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    g = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    b = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    color = std::move(cv::Scalar(b, g, r));
    cv::line(debug_frame_, pt1, pt2, color, kDebgLineWidth);
  }
  for (int i = 0; i < right_line_index.size(); ++i) {
    pt1 = cv::Point2i(
      lines[right_line_index[i]][kHoughIndex::x1],
      lines[right_line_index[i]][kHoughIndex::y1] + roi_start_height_);
    pt2 = cv::Point2i(
      lines[right_line_index[i]][kHoughIndex::x2],
      lines[right_line_index[i]][kHoughIndex::y2] + roi_start_height_);
    int r, g, b;
    r = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    g = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    b = (float)std::rand() / RAND_MAX * std::numeric_limits<uint8_t>::max();
    color = std::move(cv::Scalar(b, g, r));
    cv::line(debug_frame_, pt1, pt2, color, kDebgLineWidth);
  }
}

void HoughTransformLaneDetector::draw_rectangles(int lpos,
                                                 int rpos,
                                                 int ma_pos) {
  static cv::Scalar kCVRed(0, 0, 255);
  static cv::Scalar kCVGreen(0, 255, 0);
  static cv::Scalar kCVBlue(255, 0, 0);
  cv::rectangle(debug_frame_,
                cv::Point(lpos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(lpos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVGreen,
                kDebgLineWidth);
  cv::rectangle(debug_frame_,
                cv::Point(rpos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(rpos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVGreen,
                kDebgLineWidth);
  cv::rectangle(debug_frame_,
                cv::Point(ma_pos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(ma_pos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVRed,
                kDebgLineWidth);
  cv::rectangle(debug_frame_,
                cv::Point(image_width_ / 2 - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(image_width_ / 2 + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVBlue,
                kDebgLineWidth);
}
}  // namespace xycar