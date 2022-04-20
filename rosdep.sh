#!/bin/bash
ROS_DISTRO=${1:-noetic}
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r
rosdep update
