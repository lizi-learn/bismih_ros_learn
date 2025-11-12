cmake_minimum_required(VERSION 3.0.2)
project(ros_noetic_service_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  AddTwoInts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
add_dependencies(add_two_ints_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
add_dependencies(add_two_ints_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})

