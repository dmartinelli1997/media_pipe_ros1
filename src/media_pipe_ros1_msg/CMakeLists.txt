cmake_minimum_required(VERSION 3.0.2)
project(media_pipe_ros1_msg)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
  sensor_msgs
)


add_message_files(
  FILES
  FaceMeshPoint.msg
  HandPoint.msg
  MediaPipeHumanFaceMeshList.msg
  MediaPipeHumanHand.msg
  MediaPipeHumanHandList.msg
  MediaPipeHumanHolisticList.msg
  MediaPipeHumanPoseList.msg
  PosePoint.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


catkin_package(
   CATKIN_DEPENDS message_runtime
)

