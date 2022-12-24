/**
 * @file pid_controller.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief PID Controller Class source file
 * @version 0.2
 * @date 2022-11-27
 */
#include "lane_keeping_system/pid_controller.h"
namespace xycar {
PID::PID(float p_gain, float i_gain, float d_gain)
    : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain) {
  p_error_ = 0.0f;
  i_error_ = 0.0f;
  d_error_ = 0.0f;
}

float PID::getControlOutput(int error) {
  float float_type_error = (float)error;
  d_error_ = float_type_error - p_error_;
  p_error_ = float_type_error;
  i_error_ += float_type_error;
  return p_gain_ * p_error_ + i_gain_ * i_error_ + d_gain_ * d_error_;
}
}  // namespace xycar