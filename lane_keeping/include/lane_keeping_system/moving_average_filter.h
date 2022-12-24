/**
 * @file moving_average_filter.h
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief Moving Average Filter Class header file
 * @version 0.2
 * @date 2022-11-27
 */
#ifndef MOVING_AVERAGE_FILTER_H_
#define MOVING_AVERAGE_FILTER_H_
#include <deque>
#include <iostream>
#include <vector>

namespace xycar {
class MovingAverageFilter final {
public:
  // Construct a new Moving Average Filter object
  MovingAverageFilter(int sample_size);
  // Add new data to filter
  void addSample(int new_sample);
  // Get filtered data
  float getWeightedMovingAverage();
  // Get filtered data
  float getMovingAverage();

private:
  const int kSampleSize_;
  std::deque<int> samples_;
  std::vector<int> weight_;
};
}  // namespace xycar
#endif  // MOVING_AVERAGE_FILTER_H_
