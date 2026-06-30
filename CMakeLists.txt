# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20.2)
project(robot_body_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG TRUE)

set(PACKAGE_PUBLIC_BUILD_DEPS
  cras_cpp_common
  Eigen3
  filters
  geometric_shapes
  geometry_msgs
  laser_geometry
  moveit_ros_perception
  rclcpp
  rcl_interfaces
  rmw
  sensor_msgs
  std_srvs
  tf2_ros
  Threads
  urdf
  urdfdom_headers
  visualization_msgs
)
set(PACKAGE_PRIVATE_BUILD_DEPS
  ament_cmake
  ament_cmake_ros
  eigen_stl_containers
  pcl_conversions
  pluginlib
  rosidl_default_generators
  tf2
  tf2_eigen
  tf2_geometry_msgs
)
set(MESSAGE_BUILD_DEPS
  geometry_msgs
  std_msgs
)

foreach(dep IN ITEMS ${PACKAGE_PUBLIC_BUILD_DEPS};${PACKAGE_PRIVATE_BUILD_DEPS};${MESSAGE_BUILD_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

# Include PCL via this extras file - PCL with COMPONENTS is not supported by ament_export_dependencies() natively.
include(cmake/ConfigExtras.cmake)

# pcl_conversions introduced modern CMake target in 2.7.0 (Kilted); shim it for older distros
if(NOT TARGET pcl_conversions::pcl_conversions)
  add_library(rbf__pcl_conversions INTERFACE)
  target_include_directories(rbf__pcl_conversions INTERFACE "${pcl_conversions_INCLUDE_DIRS}")
  add_library(pcl_conversions::pcl_conversions ALIAS rbf__pcl_conversions)
endif()

ament_export_dependencies(rosidl_default_runtime)
foreach(dep IN ITEMS ${PACKAGE_PUBLIC_BUILD_DEPS};${MESSAGE_BUILD_DEPS})
  ament_export_dependencies(${dep})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/OrientedBoundingBox.msg"
  "msg/OrientedBoundingBoxStamped.msg"
  "msg/SphereStamped.msg"
  "msg/Sphere.msg"
  DEPENDENCIES ${MESSAGE_BUILD_DEPS}
  LIBRARY_NAME ${PROJECT_NAME}
  ADD_LINTER_TESTS
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}_msgs" "rosidl_typesupport_cpp")

set(UTILS_SRCS
  src/utils/bodies.cpp
  src/utils/shapes.cpp
  src/utils/tf2_eigen.cpp
)
add_library(${PROJECT_NAME}_utils ${UTILS_SRCS})
target_include_directories(${PROJECT_NAME}_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_utils ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME}_utils PUBLIC
  cras_cpp_common::cras_string_utils
  Eigen3::Eigen
  filters::filter_base
  geometric_shapes::geometric_shapes
  ${geometry_msgs_TARGETS}
  urdfdom_headers::urdfdom_headers
)
target_link_libraries(${PROJECT_NAME}_utils PRIVATE
  eigen_stl_containers::eigen_stl_containers
  rclcpp::rclcpp
)

add_library(RayCastingShapeMask src/RayCastingShapeMask.cpp)
target_include_directories(RayCastingShapeMask PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(RayCastingShapeMask PUBLIC
  Eigen3::Eigen
  geometric_shapes::geometric_shapes
  moveit_ros_perception::moveit_point_containment_filter
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)
target_link_libraries(RayCastingShapeMask PRIVATE
  ${PROJECT_NAME}_utils
  eigen_stl_containers::eigen_stl_containers
)

add_library(TFFramesWatchdog src/TfFramesWatchdog.cpp)
target_include_directories(TFFramesWatchdog PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(TFFramesWatchdog PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  Threads::Threads
)
target_link_libraries(TFFramesWatchdog PRIVATE
  ${PROJECT_NAME}_utils
  cras_cpp_common::cras_time_utils
  tf2::tf2
)

add_library(${PROJECT_NAME} src/RobotBodyFilter.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_utils
  "${cpp_typesupport_target}"
  cras_cpp_common::cras_tf2_sensor_msgs
  Eigen3::Eigen
  geometric_shapes::geometric_shapes
  ${geometry_msgs_TARGETS}
  laser_geometry::laser_geometry
  moveit_ros_perception::moveit_point_containment_filter
  pcl_common
  pcl_filters
  RayCastingShapeMask
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rmw::rmw
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  TFFramesWatchdog
  tf2_ros::tf2_ros
  Threads::Threads
  urdf::urdf
  urdfdom_headers::urdfdom_headers
  ${visualization_msgs_TARGETS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  cras_cpp_common::cras_cloud
  cras_cpp_common::cras_set_utils
  cras_cpp_common::cras_string_utils
  cras_cpp_common::cras_time_utils
  cras_cpp_common::cras_urdf_utils
  pcl_conversions::pcl_conversions
  pluginlib::pluginlib
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_library(${PROJECT_NAME}_version INTERFACE)
ament_generate_version_header(${PROJECT_NAME}_version)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME} TFFramesWatchdog RayCastingShapeMask ${PROJECT_NAME}_utils ${PROJECT_NAME}_version
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
)
install(FILES rviz/debug.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

pluginlib_export_plugin_description_file(filters laser_filters.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(cras_lint REQUIRED)
  find_package(ros_testing REQUIRED)

  cras_lint_common()
  cras_lint_cpp(SKIP_UNCRUSTIFY)
  cras_lint_py()

  ament_add_gtest(test_shapes_rbf test/test_shapes.cpp)
  target_link_libraries(test_shapes_rbf
    ${PROJECT_NAME}_utils geometric_shapes::geometric_shapes urdfdom_headers::urdfdom_headers
  )
  target_compile_definitions(test_shapes_rbf PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest(test_tf2_eigen test/test_tf2_eigen.cpp)
  target_link_libraries(test_tf2_eigen ${PROJECT_NAME}_utils Eigen3::Eigen ${geometry_msgs_TARGETS})

  ament_add_gtest(test_bodies test/test_bodies.cpp)
  target_link_libraries(test_bodies
    ${PROJECT_NAME}_utils eigen_stl_containers::eigen_stl_containers Eigen3::Eigen geometric_shapes::geometric_shapes
  )
  target_compile_definitions(test_bodies PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest_executable(test_filter_utils test/test_filter_utils.cpp)
  target_link_libraries(test_filter_utils ${PROJECT_NAME}_utils filters::filter_base rclcpp::rclcpp)
  add_ros_test(test/test_filter_utils.test.py TIMEOUT 30 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_tf_frames_watchdog test/test_tf_frames_watchdog.cpp)
  target_link_libraries(test_tf_frames_watchdog TFFramesWatchdog rclcpp::rclcpp tf2_ros::tf2_ros)
  add_ros_test(test/test_with_ros.test.py TIMEOUT 30 TARGET test_tf_frames_watchdog_rostest ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}" "binary:=test_tf_frames_watchdog" "name:=test_tf_frames_watchdog")

  ament_add_gtest_executable(test_ray_casting_shape_mask test/test_ray_casting_shape_mask.cpp)
  target_link_libraries(test_ray_casting_shape_mask
    ${PROJECT_NAME}_utils
    RayCastingShapeMask
    cras_cpp_common::cras_cloud
    Eigen3::Eigen
    geometric_shapes::geometric_shapes
    moveit_ros_perception::moveit_point_containment_filter
    rclcpp::rclcpp
    urdfdom_headers::urdfdom_headers
  )
  if("${CMAKE_BUILD_TYPE}" STREQUAL "Release" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=1)
  else()
    target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=0)
  endif()
  target_compile_definitions(test_ray_casting_shape_mask PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_ros_test(test/test_with_ros.test.py TIMEOUT 30 TARGET test_ray_casting_shape_mask_rostest ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}" "binary:=test_ray_casting_shape_mask" "name:=test_ray_casting_mask")

  ament_add_gtest_executable(test_robot_body_filter test/test_robot_body_filter.cpp)
  target_link_libraries(test_robot_body_filter
    ${PROJECT_NAME} ${cpp_typesupport_target} cras_cpp_common::cras_cloud ${geometry_msgs_TARGETS}
    moveit_ros_perception::moveit_point_containment_filter RayCastingShapeMask rclcpp::rclcpp ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS} ${visualization_msgs_TARGETS}
  )
  target_compile_options(test_robot_body_filter PRIVATE -fno-var-tracking-assignments)  # speed up build
  add_ros_test(test/test_robot_body_filter.test.py TIMEOUT 30 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  install(
    FILES test/box.dae test/triangle.dae
    DESTINATION share/robot_body_filter/test
  )
endif()

ament_package(CONFIG_EXTRAS "cmake/ConfigExtras.cmake")
