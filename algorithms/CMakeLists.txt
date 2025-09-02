cmake_minimum_required(VERSION 3.5)
project(algorithms)

set(CMAKE_CXX_STANDARD 20)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)



add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp nav_msgs geometry_msgs tf2_geometry_msgs tf2
tf2_ros
tf2_geometry_msgs)

add_executable(send_poses src/send_poses.cpp)
ament_target_dependencies(send_poses rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs tf2_geometry_msgs tf2 tf2_ros tf2_sensor_msgs tf2_ros tf2_eigen PCL Eigen3)

add_executable(bug1 src/bug1.cpp)
ament_target_dependencies(bug1 rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs tf2_geometry_msgs tf2 tf2_ros tf2_sensor_msgs tf2_ros tf2_eigen PCL Eigen3)

add_executable(create_graph src/create_graph.cpp)
ament_target_dependencies(create_graph rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs tf2_geometry_msgs tf2 tf2_ros tf2_sensor_msgs tf2_ros tf2_eigen PCL Eigen3)

add_executable(load_all_points_to_cache src/load_all_points_to_cache.cpp)
ament_target_dependencies(load_all_points_to_cache rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs )

add_executable(publish_grid_map src/publish_grid_map.cpp)
ament_target_dependencies(publish_grid_map rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs OpenCV yaml-cpp TBB )

add_executable(create_graph_with_chuncks src/create_graph_with_chuncks.cpp)
ament_target_dependencies(create_graph_with_chuncks rclcpp nav_msgs geometry_msgs PCL pcl_conversions sensor_msgs tf2_geometry_msgs tf2 tf2_ros tf2_sensor_msgs tf2_ros tf2_eigen PCL Eigen3)

target_link_libraries(send_poses
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_link_libraries(bug1
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_link_libraries(create_graph
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_link_libraries(controller
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_link_libraries(load_all_points_to_cache
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(publish_grid_map
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES} 
)

target_link_libraries(create_graph_with_chuncks
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(
 
  DIRECTORY
  
  rviz
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
  controller
  bug1
  send_poses
  create_graph
  load_all_points_to_cache
  publish_grid_map
  create_graph_with_chuncks
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()