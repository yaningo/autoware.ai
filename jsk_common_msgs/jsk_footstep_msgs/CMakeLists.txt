# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5.1)
project(jsk_footstep_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

## Collection Message Paths 
file(GLOB msg_files msg/*.msg)
message("Message Files: " ${msg_files})
set(msgs )
foreach(filePath ${msg_files})
  message("File: " ${filePath})
  get_filename_component(name ${filePath} NAME)
  message("name: " ${name})
  set(msgs ${msgs} msg/${name})
endforeach(filePath)

message("msgs: " ${msgs})

file(GLOB action_files action/*.action)
set(actions )
foreach(filePath ${action_files})
  get_filename_component(name ${filePath} NAME)
  set(actions ${actions} action/${name})
endforeach(filePath)


if(${ROS_VERSION} EQUAL 1) # ROS 1

  # Load catkin and all dependencies required for this package
  # TODO: remove all from COMPONENTS that are not catkin packages.
  find_package(catkin REQUIRED COMPONENTS geometry_msgs actionlib_msgs message_generation)

  add_message_files(
    DIRECTORY msg
    FILES ${msgs}
  )

  add_action_files(
    DIRECTORY action
    FILES ${actions}
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES geometry_msgs actionlib_msgs
  )

  # TODO: fill in what other packages will need to use this package
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
    DEPENDS
    CATKIN_DEPENDS  geometry_msgs actionlib_msgs message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
  )

else() # ROS 2

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(actionlib_msgs REQUIRED)
  find_package(action_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msgs}
      ${actions}
    DEPENDENCIES
      geometry_msgs
      actionlib_msgs
      action_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()



