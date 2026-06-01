cmake_minimum_required(VERSION 3.8)
project(b2_thesis_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(path_tracker src/path_tracker_node.cpp)
ament_target_dependencies(path_tracker rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs mocap4r2_msgs)

add_executable(mocap_to_tum src/mocap_to_tum_node.cpp)
ament_target_dependencies(mocap_to_tum rclcpp mocap4r2_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(pcd_to_2d src/pcd_to_2d.cpp)
target_include_directories(pcd_to_2d PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pcd_to_2d ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

install(TARGETS path_tracker mocap_to_tum pcd_to_2d
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  b2_thesis_fusion/lowstate_imu_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lowstate_imu_bridge
)

install(DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
