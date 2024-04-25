#!/bin/bash

MCAP_PATH="mcap-linux-amd64"
if [ ! -e $MCAP_PATH ]; then
    wget https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.44/mcap-linux-amd64
    chmod +x mcap-linux-amd64
fi

ros2 bag merge -o rosbag/rw_neo_universe/ rosbag/aligned_depth_realsense/ rosbag/color_realsense/ rosbag/depth_realsense/ rosbag/rs_depth_point/ 

./mcap-linux-amd64 convert ./rosbag/rw_neo_universe/rw_neo_universe_0.db3 ./rosbag/rw_neo_universe/rw_neo_universe_0.mcap 
