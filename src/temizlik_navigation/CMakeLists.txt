cmake_minimum_required(VERSION 3.8)
project(temizlik_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Fields2Cover REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(TARGET Fields2Cover::Fields2Cover)
  set(FIELDS2COVER_LIB_TARGET Fields2Cover::Fields2Cover)
elseif(TARGET Fields2Cover)
  set(FIELDS2COVER_LIB_TARGET Fields2Cover)
elseif(DEFINED Fields2Cover_LIBRARIES)
  set(FIELDS2COVER_LIB_TARGET ${Fields2Cover_LIBRARIES})
elseif(DEFINED fields2cover_LIBRARIES)
  set(FIELDS2COVER_LIB_TARGET ${fields2cover_LIBRARIES})
else()
  message(FATAL_ERROR "Could not resolve Fields2Cover library target")
endif()

if(DEFINED Fields2Cover_INCLUDE_DIRS)
  include_directories(${Fields2Cover_INCLUDE_DIRS})
elseif(DEFINED fields2cover_INCLUDE_DIRS)
  include_directories(${fields2cover_INCLUDE_DIRS})
endif()

add_executable(f2c_smoke_test src/f2c_smoke_test.cpp)
target_link_libraries(f2c_smoke_test PRIVATE ${FIELDS2COVER_LIB_TARGET})

add_executable(f2c_nav2_waypoint_node src/f2c_nav2_waypoint_node.cpp)
ament_target_dependencies(
  f2c_nav2_waypoint_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  std_srvs
  visualization_msgs
)
target_link_libraries(f2c_nav2_waypoint_node ${FIELDS2COVER_LIB_TARGET})

install(
  TARGETS
    f2c_smoke_test
    f2c_nav2_waypoint_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/gazebo_ground_truth_tf.py
    scripts/initial_pose_publisher.py
    scripts/opennav_coverage_waypoint_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
