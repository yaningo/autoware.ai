cmake_minimum_required(VERSION 2.8.3)
project(autoware_msgs)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)

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
            SyncTimeDiff.msg

            ## Remote Control
            RemoteCmd.msg
)

add_service_files(
  FILES RecognizeLightState.srv
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
