```bash
roslaunch franka_example_controllers cartesian_impedance_example_controller.launch \
  robot_ip:= load_gripper:=true robot:=

roslaunch franka_example_controllers joystick_pose_publisher.launch

sudo apt-get update
sudo apt-get install ros-noetic-joy
rosrun joy joy_node

```
