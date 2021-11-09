cmake_minimum_required(VERSION 2.8.3)
project(speech_recognition_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(msgs
  Grammar.msg
  PhraseRule.msg
  SpeechRecognitionCandidates.msg
  Vocabulary.msg
)

set(srvs
  SpeechRecognition.srv
)



if(${ROS_VERSION} EQUAL 1) # ROS 1

  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(
    FILES
    ${msgs}
  )

  add_service_files(
    FILES
    ${srvs}
  )

  generate_messages(
    DEPENDENCIES std_msgs
  )

  catkin_package(
      CATKIN_DEPENDS std_msgs message_runtime
  )

else() # ROS 2

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msgs}
      ${srvs}
    DEPENDENCIES
      std_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()

