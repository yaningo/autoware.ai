# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5.1)
project(posedetection_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(msgs
  Curve1D.msg
  Feature0D.msg
  Feature1D.msg
  ImageFeature0D.msg
  ImageFeature1D.msg
  Object6DPose.msg
  ObjectDetection.msg
)

set(srvs
  Detect.srv
  Feature0DDetect.srv
  Feature1DDetect.srv
  TargetObj.srv
)


# NOTE: The CPP code in this repo is only build for ROS1
if(${ROS_VERSION} EQUAL 1) # ROS 1

  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge message_generation message_filters)

  include_directories(SYSTEM include
                            ${catkin_INCLUDE_DIRS})

  add_message_files(FILES
    ${msgs}
  )

  add_service_files(FILES
    ${srvs}
  )

  # to compatible with fuerte directories
  generate_messages(
    DEPENDENCIES std_msgs sensor_msgs geometry_msgs
  )

  catkin_package(
      CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime
      DEPENDS
      INCLUDE_DIRS include
      LIBRARIES # TODO
  )

  find_package(OpenCV)
  add_executable(feature0d_view src/feature0d_view.cpp)
  target_link_libraries(feature0d_view ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(feature0d_view posedetection_msgs_gencpp)
  add_executable(feature0d_to_image src/feature0d_to_image.cpp)
  target_link_libraries(feature0d_to_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(feature0d_to_image posedetection_msgs_gencpp)

  install(TARGETS feature0d_view feature0d_to_image
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

else() # ROS 2

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msgs}
    DEPENDENCIES
      std_msgs
      sensor_msgs
      geometry_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()


