cmake_minimum_required(VERSION 3.0.2)
project(inverted_pendulum_sim)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

add_message_files(
  FILES
  CurrentState.msg
  ControlForce.msg
 )

add_service_files(
  FILES
  SetParams.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS message_generation rospy  message_runtime)
install(FILES)

