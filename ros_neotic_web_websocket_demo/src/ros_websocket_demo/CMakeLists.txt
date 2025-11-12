cmake_minimum_required(VERSION 3.0.2)
project(ros_websocket_demo)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenSSL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

add_executable(websocket_relay_node src/websocket_relay_node.cpp)

add_dependencies(websocket_relay_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(websocket_relay_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenSSL::SSL
  OpenSSL::Crypto
)

