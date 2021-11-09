# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5.1)
project(jsk_gui_msgs)


find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(msgs
  Action.msg
  MagneticField.msg
  Touch.msg
  AndroidSensor.msg
  Gravity.msg
  MultiTouch.msg
  TouchEvent.msg
  DeviceSensor.msg
  Tablet.msg
  VoiceMessage.msg
  SlackMessage.msg
)

set(srvs
  Query.srv
  YesNo.srv
)


if(${ROS_VERSION} EQUAL 1) # ROS 1

  find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs geometry_msgs)

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  add_message_files(FILES
    ${msgs}
  )

  add_service_files(FILES
    ${srvs}
  )

  generate_messages(
    DEPENDENCIES std_msgs sensor_msgs geometry_msgs
  )

  catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime
    DEPENDS
    INCLUDE_DIRS
    LIBRARIES
  )

else() # ROS 2

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msgs}
      ${srvs}
    DEPENDENCIES
      std_msgs
      sensor_msgs
      geometry_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()
