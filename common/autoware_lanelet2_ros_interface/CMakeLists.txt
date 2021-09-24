cmake_minimum_required(VERSION 2.8.11)
project(autoware_lanelet2_ros_interface)

#
#- Modified to install add CARMA regulatory elements to build
#    - 2/19/2019
#    - Michael McConnell
#

set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals")

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
  )
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)

find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

find_package(autoware_build_flags REQUIRED)

find_package(PROJ4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  amathutils_lib
  roscpp
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  lanelet2_extension
  autoware_lanelet2_msgs
  autoware_msgs
  geometry_msgs
  visualization_msgs
  roslint
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++17")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_lanelet2_ros_interface_lib
  CATKIN_DEPENDS amathutils_lib roscpp lanelet2_core lanelet2_io lanelet2_maps lanelet2_projection lanelet2_routing lanelet2_traffic_rules lanelet2_validation autoware_lanelet2_msgs autoware_msgs geometry_msgs visualization_msgs lanelet2_extension
)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
  ${PROJ4_INCLUDE_DIRS}
)

add_definitions (${GeographicLib_DEFINITIONS})

add_library( autoware_lanelet2_ros_interface_lib
  lib/message_conversion.cpp
  lib/query.cpp
  lib/utilities.cpp
  lib/visualization.cpp
)

add_dependencies(autoware_lanelet2_ros_interface_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(autoware_lanelet2_ros_interface_lib
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PROJ4_LIBRARIES}
)

add_executable(autoware_lanelet2_ros_interface_sample src/sample_code.cpp)
add_dependencies(autoware_lanelet2_ros_interface_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoware_lanelet2_ros_interface_sample
  ${catkin_LIBRARIES}
  autoware_lanelet2_ros_interface_lib
)

add_executable(autoware_lanelet2_validation src/validation.cpp)
add_dependencies(autoware_lanelet2_validation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoware_lanelet2_validation
  ${catkin_LIBRARIES}
  ${PUGIXML_LIBRAREIS}
  autoware_lanelet2_ros_interface_lib
)

install(TARGETS autoware_lanelet2_ros_interface_lib autoware_lanelet2_ros_interface_sample autoware_lanelet2_validation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
 roslint_add_test()
 find_package(rostest REQUIRED)
add_rostest_gtest(message_conversion-test test/test_message_conversion.test test/src/test_message_conversion.cpp)
target_link_libraries(message_conversion-test ${catkin_LIBRARIES} autoware_lanelet2_ros_interface_lib)
add_rostest_gtest(query-test test/test_query.test test/src/test_query.cpp)
target_link_libraries(query-test ${catkin_LIBRARIES} autoware_lanelet2_ros_interface_lib)
add_rostest_gtest(utilities-test test/test_utilities.test test/src/test_utilities.cpp)
target_link_libraries(utilities-test ${catkin_LIBRARIES} autoware_lanelet2_ros_interface_lib)

endif()
