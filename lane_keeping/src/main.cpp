/**
 * @file main.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief Lane Keeping System Main Function using Hough Transform
 * @version 0.2
 * @date 2022-11-27
 */
#include "lane_keeping_system/lane_keeping_system.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Lane Keeping System");
  xycar::LaneKeepingSystem lks;
  lks.run();

  return 0;
}