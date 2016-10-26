cmake_minimum_required(VERSION 2.4.6)
project(mc_tcp_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation geometry_msgs)

add_service_files(
  DIRECTORY srv
  FILES
  EnableController.srv
  open_grippers.srv
  close_grippers.srv
  set_gripper.srv
  get_joint_pos.srv
  set_joint_pos.srv
  play_next_stance.srv
  send_msg.srv
  send_recv_msg.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
