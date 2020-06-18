#!/bin/bash

set -x

source /home/carma/.base-image/init-env.sh
export ROS_LANG_DISABLE=genjava # Disable genjava as it is not needed in this image and makes build inconsistent 
autoware_src="/home/carma/autoware.ai"
cd ${autoware_src}
./autoware/ros/carma_autoware_build -a ${autoware_src}
