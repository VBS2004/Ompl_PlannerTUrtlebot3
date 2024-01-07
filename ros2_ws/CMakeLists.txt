cmake_minimum_required(VERSION 3.8)
project(my_robot_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ompl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_motion_planner src/mp.cpp)

target_include_directories(my_motion_planner PRIVATE
  SYSTEM /usr/include
  ${OMPL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  /root/opencv/opencv/include
  ${YAML_CPP_INCLUDE_DIR}
  
)


target_link_libraries(my_motion_planner PRIVATE
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  stdc++fs
  yaml-cpp
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

install(
  DIRECTORY src/launch
  DESTINATION share/${PROJECT_NAME}
  
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

