/**
 * @file pid_controller.h
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief PID Controller Class header file
 * @version 0.2
 * @date 2022-11-27
 */
#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

namespace xycar {
class PID final {
public:
  // Construct a new PID object
  PID(float p_gain, float i_gain, float d_gain);
  // Calculate PID control
  float getControlOutput(int error);

private:
  const float p_gain_;
  const float i_gain_;
  const float d_gain_;
  float p_error_;
  float i_error_;
  float d_error_;
};
}  // namespace xycar
#endif  // PID_CONTROLLER_H_
