source ~/ros2_lyrical/install/setup.bash
source ~/mavros_ws/install/setup.bash
source ~/boat/ros2_ws/install/setup.bash

ros2 launch boat_bringup bendyruler_stack.launch.py \
  record:=true \
  bag_prefix:=dock_test01
