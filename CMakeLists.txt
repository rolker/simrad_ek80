cmake_minimum_required(VERSION 2.8.3)
project(simrad_ek80)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  marine_acoustic_msgs
  pcl_ros
  rosbag
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
CATKIN_DEPENDS marine_acoustic_msgs pcl_ros rosbag
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

set (RAW_TO_BAG_SOURCES
  src/file/channel.cpp
  src/file/ek80.cpp
  src/file/raw_file.cpp
  src/file/raw_to_bag.cpp
  src/file/transceiver.cpp
  src/file/transducer.cpp
  src/utilities.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
    
add_executable(raw_to_bag ${RAW_TO_BAG_SOURCES})
add_dependencies(raw_to_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(raw_to_bag ${catkin_LIBRARIES})

add_executable(extract_targets src/extract_targets.cpp)
add_dependencies(extract_targets ${catkin_EXPORTED_TARGETS})
target_link_libraries(extract_targets ${catkin_LIBRARIES})
