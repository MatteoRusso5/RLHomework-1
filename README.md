# Homework 1

$ git clone https://github.com/FedericoTr26/RL_Homework-1.git

$ colcon build 

$ source install/setup.bash

## Run RViz 
$ ros2 launch arm_description display.launch.py

## Run manipulator with activated controllers in Gazebo
$ ros2 launch arm_gazebo arm_gazebo.launch.py

## Run camera on RQT
$ ros2 run rqt_image_view rqt_image_view

## Run the node to read the joint states and send joint position commands
$ ros2 run arm_controller arm_controller_node
