edit frank_ros for joystick
```bash
roslaunch franka_example_controllers cartesian_impedance_example_controller.launch \
  robot_ip:= load_gripper:=true robot:=

roslaunch franka_example_controllers joystick_pose_publisher.launch

# install
mkdir -p franka_ros_ws/src
sudo apt-get update
sudo apt-get install ros-noetic-joy
rosrun joy joy_node

sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.sh
```

https://frankaemika.github.io/docs/installation_linux.html#installing-from-the-ros-repositories
