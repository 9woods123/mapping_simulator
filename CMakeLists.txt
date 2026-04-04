cmake_minimum_required(VERSION 3.0.2)
project(mapping_simulator)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  rospy
  std_msgs
  roscpp
  octomap_msgs
  
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES fmt_ros
 CATKIN_DEPENDS mavros mavros_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(mapping_simulator src/mapping_simulator/mapping_simulator.cpp)

target_link_libraries(mapping_simulator
  ${catkin_LIBRARIES}
  ${octomap_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(mapping_simulator_node src/mapping_simulator/mapping_simulator_node.cpp)

target_link_libraries(mapping_simulator_node
mapping_simulator
)


add_library(visualization src/path_planner/visualization.cpp)
target_link_libraries(visualization
  ${catkin_LIBRARIES}
)





add_library(rrtstar_lib src/path_planner/rrtstar.cpp)
target_link_libraries(rrtstar_lib
  mapping_simulator
  visualization
)

add_executable(rrtstar src/path_planner/rrtstar.cpp)
target_link_libraries(rrtstar
  mapping_simulator
  visualization
)


add_executable(data_generation_node src/data_generation/data_generation_node.cpp)
target_link_libraries(data_generation_node
 rrtstar_lib
 mapping_simulator
)
