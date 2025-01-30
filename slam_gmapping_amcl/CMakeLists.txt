cmake_minimum_required(VERSION 3.0.2)
project(slam_gmapping_amcl)

# Add basic dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2 
  tf2_ros
  tf
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)

# Add custom services
add_service_files(
  FILES
  SaveMap.srv
)

# Generate message and service files
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Add build-time dependencies
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tf2_class save_map_class
  CATKIN_DEPENDS message_runtime
)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Add custom classes
add_library(tf2_class src/TF2Broadcaster.cpp)
add_dependencies(tf2_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf2_class ${catkin_LIBRARIES})

add_library(save_map_class src/TrajectoryPlotter.cpp)
add_dependencies(save_map_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_map_class ${catkin_LIBRARIES})

# Add executable nodes
add_executable(mapping src/mapping.cpp)
add_dependencies(mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapping ${catkin_LIBRARIES} tf2_class)

add_executable(savemap src/draw_trajectory.cpp)
add_dependencies(savemap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(savemap ${catkin_LIBRARIES} save_map_class ${OpenCV_LIBRARIES})

