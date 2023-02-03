cmake_minimum_required(VERSION 3.0.2)
project(mars_leaf_follower)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    tf
    sensor_msgs 
    std_msgs
    geometry_msgs 
    tf_conversions

)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mars_leaf_follower
   CATKIN_DEPENDS 
     roscpp 
     tf
     sensor_msgs 
     std_msgs
     geometry_msgs 
     tf_conversions
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories("/usr/local/include/eigen3")

add_executable(${PROJECT_NAME}_follower_node src/follower_node.cpp)
set_target_properties(${PROJECT_NAME}_follower_node PROPERTIES OUTPUT_NAME follower_node PREFIX "")
add_dependencies(${PROJECT_NAME}_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_follower_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_grasp_node src/grasp_control_node.cpp)
set_target_properties(${PROJECT_NAME}_grasp_node PROPERTIES OUTPUT_NAME grasp_control_node PREFIX "")
add_dependencies(${PROJECT_NAME}_grasp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_grasp_node
  ${catkin_LIBRARIES}
)
