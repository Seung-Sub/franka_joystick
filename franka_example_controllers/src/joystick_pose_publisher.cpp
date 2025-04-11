// joystick_pose_publisher.cpp
#include <franka_example_controllers/joystick_pose_publisher.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/topic.h>

namespace franka_example_controllers {

JoystickPosePublisher::JoystickPosePublisher(ros::NodeHandle &nh)
    : nh_(nh),
      target_x_(0.0), target_y_(0.0), target_z_(0.0),
      joy_delta_x_(0.0), joy_delta_y_(0.0), joy_delta_z_(0.0),
      scale_x_(0.01), scale_y_(0.01), scale_z_(0.01),
      initial_orientation_w_(1.0), initial_orientation_x_(0.0),
      initial_orientation_y_(0.0), initial_orientation_z_(0.0),
      init_x_(0.0), init_y_(0.0), init_z_(0.0),
      gripper_open_width_(0.08), gripper_open_speed_(0.1),
      gripper_close_width_(0.03), gripper_close_speed_(0.1), gripper_close_force_(5.0),
      gripper_close_epsilon_inner_(0.005), gripper_close_epsilon_outer_(0.005),
      last_buttons_(5, 0)  // 최소 3번 버튼까지 접근 가능하도록 크기 5로 초기화
{
  // 조이스틱 감도 파라미터
  nh_.param("scale_x", scale_x_, scale_x_);
  nh_.param("scale_y", scale_y_, scale_y_);
  nh_.param("scale_z", scale_z_, scale_z_);

  // 그리퍼 파라미터
  nh_.param("gripper_open_width",  gripper_open_width_,  gripper_open_width_);
  nh_.param("gripper_open_speed",  gripper_open_speed_,  gripper_open_speed_);
  nh_.param("gripper_close_width", gripper_close_width_, gripper_close_width_);
  nh_.param("gripper_close_speed", gripper_close_speed_, gripper_close_speed_);
  nh_.param("gripper_close_force", gripper_close_force_, gripper_close_force_);
  nh_.param("gripper_close_epsilon_inner", gripper_close_epsilon_inner_, gripper_close_epsilon_inner_);
  nh_.param("gripper_close_epsilon_outer", gripper_close_epsilon_outer_, gripper_close_epsilon_outer_);

  // 초기 EE 포즈 읽기
  waitForInitialPose();

  // Interactive Marker feedback 퍼블리셔
  feedback_pub_ = nh_.advertise<visualization_msgs::InteractiveMarkerFeedback>(
      "/equilibrium_pose_marker/feedback", 10);

  // 그리퍼 액션 goal 퍼블리셔
  gripper_move_pub_  = nh_.advertise<franka_gripper::MoveActionGoal>(
      "/franka_gripper/move/goal", 1);
  gripper_grasp_pub_ = nh_.advertise<franka_gripper::GraspActionGoal>(
      "/franka_gripper/grasp/goal", 1);

  // 조이스틱 구독
  joy_sub_ = nh_.subscribe("/joy", 10, &JoystickPosePublisher::joyCallback, this);

  // 주기적 퍼블리시 타이머
  timer_ = nh_.createTimer(ros::Duration(0.05), &JoystickPosePublisher::timerCallback, this);

  ROS_INFO("JoystickPosePublisher node started.");
}

void JoystickPosePublisher::waitForInitialPose() {
  ROS_INFO("Waiting for initial FrankaState message (5s timeout)...");
  auto state_msg = ros::topic::waitForMessage<franka_msgs::FrankaState>(
      "franka_state_controller/franka_states", nh_, ros::Duration(5.0));
  if (state_msg && state_msg->O_T_EE.size() >= 16) {
    Eigen::Matrix4d O_T_EE;
    for (int i = 0; i < 16; ++i) O_T_EE(i/4, i%4) = state_msg->O_T_EE[i];
    O_T_EE.transposeInPlace();

    target_x_ = init_x_ = O_T_EE(0,3);
    target_y_ = init_y_ = O_T_EE(1,3);
    target_z_ = init_z_ = O_T_EE(2,3);
    if (std::fabs(target_x_)<1e-6 && std::fabs(target_y_)<1e-6 && std::fabs(target_z_)<1e-6) {
      ROS_WARN("FrankaState translation near zero, using parameters.");
      nh_.param("initial_x", target_x_, target_x_);
      nh_.param("initial_y", target_y_, target_y_);
      nh_.param("initial_z", target_z_, target_z_);
      init_x_ = target_x_;
      init_y_ = target_y_;
      init_z_ = target_z_;
    }
    Eigen::Quaterniond q(Eigen::Matrix3d(O_T_EE.block<3,3>(0,0)));
    q.normalize();
    initial_orientation_w_ = q.w();
    initial_orientation_x_ = q.x();
    initial_orientation_y_ = q.y();
    initial_orientation_z_ = q.z();
    ROS_INFO_STREAM("Initial EE pos: ["<<target_x_<<","<<target_y_<<","<<target_z_<<"]");
    ROS_INFO_STREAM("Initial EE ori: ["<<initial_orientation_w_<<","<<initial_orientation_x_
                    <<","<<initial_orientation_y_<<","<<initial_orientation_z_<<"]");
  } else {
    ROS_WARN("No valid FrankaState, using fallback params.");
    nh_.param("initial_x", target_x_, init_x_);
    nh_.param("initial_y", target_y_, init_y_);
    nh_.param("initial_z", target_z_, init_z_);
    nh_.param("initial_orientation_w", initial_orientation_w_, initial_orientation_w_);
    nh_.param("initial_orientation_x", initial_orientation_x_, initial_orientation_x_);
    nh_.param("initial_orientation_y", initial_orientation_y_, initial_orientation_y_);
    nh_.param("initial_orientation_z", initial_orientation_z_, initial_orientation_z_);
    init_x_ = target_x_;
    init_y_ = target_y_;
    init_z_ = target_z_;
    ROS_INFO_STREAM("Fallback EE pos: ["<<target_x_<<","<<target_y_<<","<<target_z_<<"]");
    ROS_INFO_STREAM("Fallback EE ori: ["<<initial_orientation_w_<<","<<initial_orientation_x_
                    <<","<<initial_orientation_y_<<","<<initial_orientation_z_<<"]");
  }
}

void JoystickPosePublisher::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  if (msg->axes.size()<4 || msg->buttons.size()<4) {
    ROS_WARN("Joy message too small (axes=%lu, buttons=%lu).",
             msg->axes.size(), msg->buttons.size());
    return;
  }
  // 축 입력
  joy_delta_x_ = msg->axes[1] * scale_x_;
  joy_delta_y_ = msg->axes[0] * scale_y_;
  joy_delta_z_ = msg->axes[3] * scale_z_;

  // 버튼 0 (index 0): 그리퍼 닫기 → grasp
  if (msg->buttons[0]==1 && last_buttons_[0]==0) {
    franka_gripper::GraspActionGoal goal;
    goal.goal.width   = gripper_close_width_;
    goal.goal.epsilon.inner = gripper_close_epsilon_inner_;
    goal.goal.epsilon.outer = gripper_close_epsilon_outer_;
    goal.goal.speed   = gripper_close_speed_;
    goal.goal.force   = gripper_close_force_;
    gripper_grasp_pub_.publish(goal);
    ROS_INFO("Gripper grasp goal sent (width=%.3f).", gripper_close_width_);
  }
  // 버튼 2 (index 2): 그리퍼 열기 → move
  if (msg->buttons[2]==1 && last_buttons_[2]==0) {
    franka_gripper::MoveActionGoal goal;
    goal.goal.width = gripper_open_width_;
    goal.goal.speed = gripper_open_speed_;
    gripper_move_pub_.publish(goal);
    ROS_INFO("Gripper move goal sent (width=%.3f).", gripper_open_width_);
  }

  // 버튼 3 (index 3): 기존 리셋 기능
  if (msg->buttons[4]==1 && last_buttons_[3]==0) {
    ROS_INFO("Button 4 pressed: reset EE pos to init.");
    target_x_ = init_x_;
    target_y_ = init_y_;
    target_z_ = init_z_;
  }

  // 다음 엣지 검출을 위해 저장
  last_buttons_[0] = msg->buttons[0];
  last_buttons_[2] = msg->buttons[2];
  last_buttons_[3] = msg->buttons[3];
}

void JoystickPosePublisher::timerCallback(const ros::TimerEvent &) {
  // 조이스틱 변화량 누적
  target_x_ += joy_delta_x_;
  target_y_ += joy_delta_y_;
  target_z_ += joy_delta_z_;

  // InteractiveMarkerFeedback 메시지
  visualization_msgs::InteractiveMarkerFeedback fb;
  fb.header.stamp = ros::Time::now();
  fb.header.frame_id = "panda_link0";
  fb.client_id    = "joystick";
  fb.marker_name  = "equilibrium_pose";
  fb.control_name = "move_x";
  fb.event_type   = visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;

  fb.pose.position.x = target_x_;
  fb.pose.position.y = target_y_;
  fb.pose.position.z = target_z_;
  fb.pose.orientation.w = initial_orientation_w_;
  fb.pose.orientation.x = initial_orientation_x_;
  fb.pose.orientation.y = initial_orientation_y_;
  fb.pose.orientation.z = initial_orientation_z_;

  fb.menu_entry_id    = 0;
  fb.mouse_point_valid = false;

  feedback_pub_.publish(fb);
}

}  // namespace franka_example_controllers

int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_pose_publisher");
  ros::NodeHandle nh;
  franka_example_controllers::JoystickPosePublisher publisher(nh);
  ros::spin();
  return 0;
}
















