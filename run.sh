#!/bin/bash
source devel/setup.bash
GUI=${1:-0}
#Terminal1
roslaunch flocking swarm.launch population:=10 steps:=500 world_name:=custom.world gui:=$GUI

#roslaunch flocking swarm.launch population:=5 steps:=500 world_name:=animated_cafe.world gui:=true
#roslaunch flocking swarm.launch population:=5 steps:=500 world_name:=animated_cafe.world gui:=true
#roslaunch flocking swarm.launch population:=5 steps:=500 world_name:=cafe.world gui:=true
#roslaunch flocking swarm.launch population:=5 steps:=500 world_name:=no_roof_small_warehouse.world gui:=true
#roslaunch flocking swarm.launch population:=5 steps:=500 world_name:=clinic.world gui:=true
#roslaunch flocking swarm.launch population:=5 steps:=500 world_name:=empty_sky.world gui:=true
#roslaunch flocking swarm.launch population:=5 steps:=500 world_name:=empty.world gui:=true
#roslaunch flocking swarm.launch population:=20 steps:=500 world_name:=swarm_world.world gui:=false

#Terminal2
#roslaunch darknet_ros my_yolov3.launch config_file:=yolov3-spp image:=/stereo_camera/left/image_raw

# roslaunch darknet_ros my_yolov3.launch config_file:=yolov3-spp image:=/image_raw_top_left
# roslaunch darknet_ros my_yolov3.launch config_file:=yolov3-spp image:=/usb_cam/image_raw
# roslaunch darknet_ros my_yolov3.launch config_file:=yolov3-tiny image:=/usb_cam/image_raw
# roslaunch darknet_ros yolo_v3.launch image:=/usb_cam/image_raw

# ROS_NAMESPACE=/stereo/camera rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True
# rosrun image_view stereo_view stereo:=/stereo/camera image:=image_raw _approximate_sync:=True

#gazebo worlds
#/usr/share/gazebo-9/worlds
