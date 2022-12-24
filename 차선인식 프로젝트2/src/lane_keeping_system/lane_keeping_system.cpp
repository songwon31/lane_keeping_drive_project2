/**
 * @file lane_keeping_system.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief Lane Keeping System Class source file
 * @version 0.2
 * @date 2022-11-27
 */
#include "lane_keeping_system/lane_keeping_system.h"

namespace xycar {
LaneKeepingSystem::LaneKeepingSystem() {
  std::string config_path;
  nh_.getParam("config_path", config_path);
  // std::cout << config_path << std::endl;
  YAML::Node config = YAML::LoadFile(config_path);
  // std::cout << config << std::endl;

  float p_gain, i_gain, d_gain;
  int sample_size;
  pid_ptr_ = new PID(config["PID"]["P_GAIN"].as<float>(),
                     config["PID"]["I_GAIN"].as<float>(),
                     config["PID"]["D_GAIN"].as<float>());
  ma_filter_ptr_ = new MovingAverageFilter(
    config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<int>());
  hough_transform_lane_detector_ptr_ = new HoughTransformLaneDetector(config);
  setParams(config);

  pub_ = nh_.advertise<xycar_msgs::xycar_motor>(pub_topic_name_, queue_size_);
  sub_ = nh_.subscribe(
    sub_topic_name_, queue_size_, &LaneKeepingSystem::imageCallback, this);
}

void LaneKeepingSystem::setParams(const YAML::Node &config) {
  pub_topic_name_ = config["TOPIC"]["PUB_NAME"].as<std::string>();
  sub_topic_name_ = config["TOPIC"]["SUB_NAME"].as<std::string>();
  queue_size_ = config["TOPIC"]["QUEUE_SIZE"].as<int>();

  xycar_speed_ = config["XYCAR"]["START_SPEED"].as<float>();
  xycar_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<float>();
  xycar_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<float>();
  xycar_speed_control_threshold_ =
    config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
  acceleration_step_ = config["XYCAR"]["ACCELERATION_STEP"].as<float>();
  deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<float>();
  debug_ = config["DEBUG"].as<bool>();
}

LaneKeepingSystem::~LaneKeepingSystem() {
  delete ma_filter_ptr_;
  delete pid_ptr_;
  delete hough_transform_lane_detector_ptr_;
}

void LaneKeepingSystem::run() {
  int lpos, rpos, error, ma_mpos;
  float steering_angle;
  ros::Rate rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    if (frame_.empty()) {
      continue;
    }

    std::tie(lpos, rpos) =
      hough_transform_lane_detector_ptr_->getLanePosition(frame_);

    ma_filter_ptr_->addSample((lpos + rpos) / 2);
    ma_mpos = ma_filter_ptr_->getWeightedMovingAverage();
    error = ma_mpos - frame_.cols / 2;
    steering_angle = std::max(-(float)kXycarSteeringAngleLimit,
                              std::min(pid_ptr_->getControlOutput(error),
                                       (float)kXycarSteeringAngleLimit));

    speed_control(steering_angle);
    drive(steering_angle);

    if (debug_) {
      std::cout << "lpos: " << lpos << ", rpos: " << rpos << ", mpos: " << ma_mpos << std::endl;
      hough_transform_lane_detector_ptr_->draw_rectangles(lpos, rpos, ma_mpos);
      cv::imshow("debug",
                 *(hough_transform_lane_detector_ptr_->getDebugFrame()));
      cv::waitKey(1);
    }
    // rate.sleep();
  }
}

void LaneKeepingSystem::imageCallback(const sensor_msgs::Image &msg) {
  cv::Mat src = cv::Mat(msg.height,
                        msg.width,
                        CV_8UC3,
                        const_cast<uint8_t *>(&msg.data[0]),
                        msg.step);
  cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void LaneKeepingSystem::speed_control(float steering_angle) {
  if (std::abs(steering_angle) > xycar_speed_control_threshold_) {
    xycar_speed_ -= deceleration_step_;
    xycar_speed_ = std::max(xycar_speed_, xycar_min_speed_);
  } else {
    xycar_speed_ += acceleration_step_;
    xycar_speed_ = std::min(xycar_speed_, xycar_max_speed_);
  }
}

void LaneKeepingSystem::drive(float steering_angle) {
  xycar_msgs::xycar_motor motor_msg;
  motor_msg.angle = std::round(steering_angle);
  motor_msg.speed = std::round(xycar_speed_);

  pub_.publish(motor_msg);
}
}  // namespace xycar