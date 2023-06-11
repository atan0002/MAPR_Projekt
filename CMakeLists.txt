cmake_minimum_required(VERSION 3.8)
project(mapr_projekt2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)
#find_package(rosbag REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ompl REQUIRED)

# moveit_package()

message("ompl libraries: ${OMPL_FOUND}")

include_directories(
  include
  /opt/ros/humble/include
  /opt/ros/humble/include/urdf
  /opt/ros/humble/include/moveit_msgs
  /opt/ros/humble/include/geometry_msgs
  /opt/ros/humble/include/trajectory_msgs
  /opt/ros/humble/include/sensor_msgs
  /opt/ros/humble/include/visualization_msgs
  /opt/ros/humble/include/object_recognition_msgs
  /opt/ros/humble/include/shape_msgs
  /opt/ros/humble/include/octomap_msgs
  /opt/ros/humble/include/unique_identifier_msgs
  /opt/ros/humble/include/action_msgs
  /opt/ros/humble/include/rclcpp_action
  /opt/ros/humble/include/rcl_action
  /opt/ros/humble/include/tf2_msgs
  /opt/ros/humble/include/pluginlib
  /opt/ros/humble/include/class_loader
  /opt/ros/humble/include/graph_msgs
  /opt/ros/humble/include/ompl-1.6
  ${EIGEN3_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)




add_executable(plan_trajectory src/plan_trajectory.cpp)
target_include_directories(plan_trajectory PUBLIC include)
ament_target_dependencies(plan_trajectory moveit_common moveit_core moveit_ros_planning moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools rclcpp std_msgs ompl) #
target_link_libraries(plan_trajectory ${OMPL_LIBRARIES})


add_executable(plan_trajectory_with_viz src/plan_trajectory_with_viz.cpp)
target_include_directories(plan_trajectory_with_viz PUBLIC include)
ament_target_dependencies(plan_trajectory_with_viz moveit_common moveit_core moveit_ros_planning moveit_ros_planning_interface rviz_visual_tools moveit_visual_tools rclcpp std_msgs ompl) #
target_link_libraries(plan_trajectory_with_viz ${OMPL_LIBRARIES})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # set(“-L/usr/local/lib -lompl”)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS plan_trajectory DESTINATION lib/${PROJECT_NAME})
install(TARGETS plan_trajectory_with_viz DESTINATION lib/${PROJECT_NAME})

ament_package()


