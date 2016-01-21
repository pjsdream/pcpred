#!/bin/bash
if [[ $# -ne 2 ]]; then
  echo "Usage: ./record.sh (#frames) (#sequence)"
else
  echo "5"
  sleep 1
  echo "4"
  sleep 1
  echo "3"
  sleep 1
  echo "2"
  sleep 1
  echo "1"
  sleep 1
  echo "recording $1 frames for sequence $2"
  rosbag record camera/depth/image_raw camera/depth/camera_info --limit=$1 -O kinect$2
  rosbag info kinect$2.bag
fi
