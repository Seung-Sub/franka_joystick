// joystick_pose_publisher.hpp
#ifndef FRANKA_EXAMPLE_CONTROLLERS_JOYSTICK_POSE_PUBLISHER_HPP_
#define FRANKA_EXAMPLE_CONTROLLERS_JOYSTICK_POSE_PUBLISHER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_msgs/FrankaState.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

// gripper action goal 메시지
#include <franka_gripper/MoveActionGoal.h>
#include <franka_gripper/GraspActionGoal.h>

namespace franka_example_controllers {

class JoystickPosePublisher {
public:
  explicit JoystickPosePublisher(ros::NodeHandle &nh);

private:
  // 초기 EE 포즈와 orientation를 FrankaState 메시지에서 받아오되,
  // translation 값이 0이면 사용자 정의 파라미터값을 사용
  void waitForInitialPose();

  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void timerCallback(const ros::TimerEvent &);

  ros::NodeHandle nh_;
  ros::Publisher feedback_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer_;

  // 목표 position (누적 업데이트될 값)
  double target_x_, target_y_, target_z_;
  // 조이스틱 입력 delta 값
  double joy_delta_x_, joy_delta_y_, joy_delta_z_;
  // 조이스틱 감도 (파라미터)
  double scale_x_, scale_y_, scale_z_;

  // 초기 orientation 값 (FrankaState 메시지로부터 또는 파라미터 fallback)
  double initial_orientation_w_, initial_orientation_x_;
  double initial_orientation_y_, initial_orientation_z_;

  // 버튼 리셋 시 사용할 초기 position 저장
  double init_x_, init_y_, init_z_;

  // **gripper 제어용 퍼블리셔**
  ros::Publisher gripper_move_pub_;
  ros::Publisher gripper_grasp_pub_;

  // **그리퍼 파라미터** (모두 파라미터 서버에서 읽음)
  double gripper_open_width_, gripper_open_speed_;
  double gripper_close_width_, gripper_close_speed_, gripper_close_force_;
  double gripper_close_epsilon_inner_, gripper_close_epsilon_outer_;

  // 버튼 엣지 검출을 위한 이전 버튼 상태 저장
  std::vector<int> last_buttons_;
};

}  // namespace franka_example_controllers

#endif  // FRANKA_EXAMPLE_CONTROLLERS_JOYSTICK_POSE_PUBLISHER_HPP_




















