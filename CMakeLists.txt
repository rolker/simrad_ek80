cmake_minimum_required(VERSION 2.8.3)
project(simrad_ek80)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  acoustic_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
CATKIN_DEPENDS acoustic_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set (SOURCES
  src/channel.cpp
  src/client.cpp
  src/connection.cpp
  src/packet.cpp
  src/parameter.cpp
  src/parameter_manager.cpp
  src/request.cpp
  src/response.cpp
  src/sample.cpp
  src/server.cpp
  src/server_manager.cpp
  src/simrad_ek80_node.cpp
  src/subscription.cpp
  src/subscription_manager.cpp
  src/udp_socket.cpp
  src/utilities.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
