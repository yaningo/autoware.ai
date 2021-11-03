cmake_minimum_required(VERSION 2.8.3)
project(autoware_build_flags)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS autoware_build_flags-extras.cmake)
