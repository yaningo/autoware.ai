cmake_minimum_required(VERSION 3.5.1)
project(autoware_msgs)

find_package(carma_cmake_common REQUIRED)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(msg_files
  ControlCommandStamped.msg
  CloudCluster.msg
  CloudClusterArray.msg
  ColorSet.msg
  ControlCommand.msg
  DetectedObject.msg
  DetectedObjectArray.msg
  ExtractedPosition.msg
  ImageLaneObjects.msg
  ImageObjects.msg
  LaneArray.msg
  PointsImage.msg
  ScanImage.msg
  Signals.msg
  TunedResult.msg
  ValueSet.msg
  Centroids.msg
  DTLane.msg
  GeometricRectangle.msg
  ICPStat.msg
  ImageObj.msg
  ImageObjRanged.msg
  ImageObjTracked.msg
  ImageRect.msg
  ImageRectRanged.msg
  Lane.msg
  NDTStat.msg
  ObjLabel.msg
  ObjPose.msg
  ProjectionMatrix.msg
  VscanTracked.msg
  VscanTrackedArray.msg
  Waypoint.msg
  WaypointState.msg
  VehicleCmd.msg
  VehicleLocation.msg
  VehicleStatus.msg
  TrafficLightResult.msg
  TrafficLightResultArray.msg
  Gear.msg
  AccelCmd.msg
  AdjustXY.msg
  BrakeCmd.msg
  IndicatorCmd.msg
  LampCmd.msg
  SteerCmd.msg
  TrafficLight.msg
  StateCmd.msg
  State.msg
  ## Sync
  SyncTimeMonitor.msg

  ## Remote Control
  RemoteCmd.msg
)

set(srv_files
  RecognizeLightState.srv
)

if(${ROS_VERSION} EQUAL 1) # ROS 1

  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    jsk_recognition_msgs
    message_generation
    sensor_msgs
    std_msgs
  )

  add_message_files(
        DIRECTORY msg
        FILES
          ${msg_files}
  )

  add_service_files(
    FILES ${srv_files}
  )

  generate_messages(
    DEPENDENCIES
      geometry_msgs
      jsk_recognition_msgs
      sensor_msgs
      std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      geometry_msgs
      jsk_recognition_msgs
      message_runtime
      sensor_msgs
      std_msgs
  )

else() # ROS2

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(jsk_recognition_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msg_files}
      ${srv_files}
    DEPENDENCIES
      std_msgs
      builtin_interfaces
      jsk_recognition_msgs
      sensor_msgs
      geometry_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()

