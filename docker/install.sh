#!/bin/bash

autoware_src="/home/carma/autoware.ai"
AUTOWARE_PACKAGE_SELECTION="map_tools lidar_localizer map_file deadreckoner points_downsampler points_preprocessor lane_planner waypoint_maker autoware_msgs autoware_config_msgs as"
MSGS_TO_GENERATE="jsk_footstep_msgs jsk_recognition_msgs autoware_msgs radar_msgs derived_object_msgs autoware_config_msgs"

echo "Building Autoware required packages ${AUTOWARE_PACKAGE_SELECTION}"
cd ${autoware_src}/ros

source /opt/ros/kinetic/setup.sh
./colcon_release --executor sequential --packages-up-to "${AUTOWARE_PACKAGE_SELECTION}"
source ./install/setup.bash
echo "Autoware built successfuly. Binaries sourced from $(realpath ./install/setup.bash)"


###
# Generate rosjava message artifacts for autoware
###

# Set environment variables required by genjava_message_artifacts 
# The default values do not resolve correctly with autoware's colon build
export ROS_MAVEN_DEPLOYMENT_REPOSITORY_PREV=$ROS_MAVEN_DEPLOYMENT_REPOSITORY
export ROS_MAVEN_DEPLOYMENT_REPOSITORY="${autoware_src}/ros/install/share/maven"
export ROS_MAVEN_PATH_PREV=${ROS_MAVEN_PATH}
export ROS_MAVEN_PATH="${ROS_MAVEN_PATH}:${autoware_src}/ros/install/share/maven"

# Generate message artifacts
echo "Trying to generate rosjava message artifacts for ${MSGS_TO_GENERATE}"
genjava_message_artifacts -p ${MSGS_TO_GENERATE}

# Copy jar and pom files to the directories where gradle will search for them during the carma build
JAR_DIR="${ROS_MAVEN_DEPLOYMENT_REPOSITORY}"

for message in ${MSGS_TO_GENERATE}
do
  JAR_DEST=${autoware_src}/ros/install/${message}/share/maven
  mkdir -p ${JAR_DEST}
  cp -R ${JAR_DIR}/* ${JAR_DEST}
done

# Reset maven deployment environment variable to ensure carma builds its java messages correctly
export ROS_MAVEN_DEPLOYMENT_REPOSITORY=$ROS_MAVEN_DEPLOYMENT_REPOSITORY_PREV
export ROS_MAVEN_PATH=$ROS_MAVEN_PATH_PREV
