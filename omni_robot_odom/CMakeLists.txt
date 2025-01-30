cmake_minimum_required(VERSION 3.0.2)
project(omni_robot_odom)

# Add basic dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  message_generation
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2 
  tf2_ros
)
# find_package(Ceres REQUIRED)

# Add custom messages
add_message_files(
  FILES
  Custom2DPose.msg
  WheelSpeeds.msg
)

# Add custom services
add_service_files(
  FILES
  ResetPose.srv
)

# Generate message and service files
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Enable dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

# Add build-time dependencies
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinematic_class odometry_class tf2broad_class wheelcontrol_class
  CATKIN_DEPENDS message_runtime
)
include_directories(include ${catkin_INCLUDE_DIRS})
# include_directories(include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

# Add custom classes
add_library(kinematic_class src/KinematicsPubSub.cpp)
add_dependencies(kinematic_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic_class ${catkin_LIBRARIES})

add_library(odometry_class src/OdomPubSub.cpp)
add_dependencies(odometry_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry_class ${catkin_LIBRARIES})

add_library(tf2broad_class src/TF2Broadcaster.cpp)
add_dependencies(tf2broad_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf2broad_class ${catkin_LIBRARIES})

add_library(wheelcontrol_class src/WheelControlPubSub.cpp)
add_dependencies(wheelcontrol_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelcontrol_class ${catkin_LIBRARIES})

# Add executable nodes
add_executable(kinematics src/kinematics.cpp)
add_dependencies(kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematics ${catkin_LIBRARIES} kinematic_class)

add_executable(odometry src/odom.cpp)
add_dependencies(odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES} odometry_class tf2broad_class)

add_executable(wheelcontrol src/wheelcontrol.cpp)
add_dependencies(wheelcontrol ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelcontrol ${catkin_LIBRARIES} wheelcontrol_class)

add_executable(resetclient src/resetclient.cpp)
add_dependencies(resetclient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(resetclient ${catkin_LIBRARIES})

add_executable(path_publisher src/path_publisher.cpp)
add_dependencies(path_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_publisher ${catkin_LIBRARIES})

add_executable(initialization src/initialization.cpp)
add_dependencies(initialization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(initialization ${catkin_LIBRARIES})

# add_executable(calibration src/calibration.cpp)
# add_dependencies(calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(calibration ${catkin_LIBRARIES} ${CERES_LIBRARIES})