cmake_minimum_required(VERSION 3.7.2)
project(robot_body_filter)

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(DetectOptional)

if(${ROBOT_BODY_FILTER_HAVE_CXX_OPTIONAL})
  # add_compile_definitions would be nicer, but isn't available in Stretch
  add_compile_options(-DROBOT_BODY_FILTER_USE_CXX_OPTIONAL=1)
else()
  add_compile_options(-DROBOT_BODY_FILTER_USE_CXX_OPTIONAL=0)
endif()

set(THIS_PACKAGE_DEPS backward_ros urdf urdf_parser_plugin filters geometric_shapes laser_geometry moveit_core moveit_ros_perception rclcpp sensor_msgs std_srvs pcl_conversions tf2 tf2_ros tf2_eigen tf2_sensor_msgs urdf visualization_msgs GLUT OpenCV)
set(MESSAGE_DEPS geometry_msgs std_msgs)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPS})
        find_package(${Dependency} REQUIRED)
endforeach()

find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}_msgs "msg/OrientedBoundingBox.msg" "msg/OrientedBoundingBoxStamped.msg" DEPENDENCIES ${MESSAGE_DEPS} LIBRARY_NAME ${PROJECT_NAME})

include_directories(include include/${PROJECT_NAME} include/${PROJECT_NAME}/utils)

set(UTILS_SRCS
  src/utils/bodies.cpp
  src/utils/cloud.cpp
  src/utils/shapes.cpp
  src/utils/string_utils.cpp
  src/utils/tf2_eigen.cpp
  src/utils/tf2_sensor_msgs.cpp
  src/utils/time_utils.cpp
)
add_library(${PROJECT_NAME}_utils ${UTILS_SRCS})
target_link_libraries(${PROJECT_NAME}_utils ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_utils ${THIS_PACKAGE_DEPS})
ament_export_libraries(${PROJECT_NAME}_utils)

add_library(tf2_sensor_msgs_rbf src/utils/tf2_sensor_msgs.cpp)
target_link_libraries(tf2_sensor_msgs_rbf ${PROJECT_NAME}_utils)
ament_export_libraries(tf2_sensor_msgs_rbf)

add_library(RayCastingShapeMask src/RayCastingShapeMask.cpp)
target_link_libraries(RayCastingShapeMask ${PROJECT_NAME}_utils)

add_library(TFFramesWatchdog src/TFFramesWatchdog.cpp)
target_link_libraries(TFFramesWatchdog ${PROJECT_NAME}_utils)

add_library(${PROJECT_NAME} src/RobotBodyFilter.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_utils
  tf2_sensor_msgs_rbf
  TFFramesWatchdog
  RayCastingShapeMask
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

install(DIRECTORY include/
   DESTINATION include)

install(TARGETS ${PROJECT_NAME} TFFramesWatchdog RayCastingShapeMask tf2_sensor_msgs_rbf ${PROJECT_NAME}_utils
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(FILES rviz/debug.rviz
  DESTINATION share/rviz)

if (BUILD_TESTING) 
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_set_utils test/test_set_utils.cpp)
  target_link_libraries(test_set_utils ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_set_utils ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_shapes_rbf test/test_shapes.cpp)
  target_link_libraries(test_shapes_rbf ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_shapes_rbf ${THIS_PACKAGE_DEPS})

  # ament_add_gtest(test_urdf_eigen test/test_urdf_eigen.cpp)
  # target_link_libraries(test_urdf_eigen ${PROJECT_NAME}_utils)

#   ament_add_gtest(test_time_utils test/test_time_utils.cpp)
#   target_link_libraries(test_time_utils ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

#   ament_add_gtest(test_tf2_eigen test/test_tf2_eigen.cpp)
#   target_link_libraries(test_tf2_eigen ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

#   ament_add_gtest(test_string_utils test/test_string_utils.cpp)
#   target_link_libraries(test_string_utils ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  # ament_add_gtest(test_bodies test/test_bodies.cpp)
  # ament_target_dependencies(test_bodies ${THIS_PACKAGE_DEPS})
  # target_link_libraries(test_bodies ${PROJECT_NAME} ${PROJECT_NAME}_utils)

#   ament_add_gtest(test_cloud test/test_cloud.cpp)
#   target_link_libraries(test_cloud ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

#   ament_add_gtest(test_tf2_sensor_msgs test/test_tf2_sensor_msgs.cpp)
#   target_link_libraries(test_tf2_sensor_msgs tf2_sensor_msgs_rbf ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

#   ament_add_gtest(test_xmlrpc_traits test/test_xmlrpc_traits.cpp)
#   target_link_libraries(test_xmlrpc_traits ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

#   ament_add_gtest(test_filter_utils test/test_filter_utils.test test/test_filter_utils.cpp)
#   ament_target_dependencies(test_filter_utils ${THIS_PACKAGE_DEPS})
#   target_link_libraries(test_filter_utils lib${PROJECT_NAME})

#   ament_add_gtest(test_tf_frames_watchdog test/test_tf_frames_watchdog.cpp)
#   target_link_libraries(test_tf_frames_watchdog TFFramesWatchdog ${catkin_LIBRARIES})

#   ament_add_gtest(test_ray_casting_shape_mask test/test_ray_casting_shape_mask.cpp)
#   target_link_libraries(test_ray_casting_shape_mask RayCastingShapeMask ${catkin_LIBRARIES})
#   if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
#     target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=1)
#   else()
#     target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=0)
#   endif()

#   ament_add_gtest(test_robot_body_filter test/test_robot_body_filter.test test/test_robot_body_filter.cpp)
#   target_link_libraries(test_robot_body_filter ${PROJECT_NAME} ${catkin_LIBRARIES})
#   target_compile_options(test_robot_body_filter PRIVATE -fno-var-tracking-assignments)  # speed up build

  install(FILES test/box.dae test/triangle.dae
    DESTINATION share/robot_body_filter/test)
endif()

ament_export_include_directories(include include/${PROJECT_NAME} include/${PROJECT_NAME}/utils)
ament_export_dependencies(${THIS_PACKAGE_DEPS} rosidl_default_runtime)
ament_export_libraries(${PROJECT_NAME} TFFramesWatchdog RayCastingShapeMask tf2_sensor_msgs_rbf ${PROJECT_NAME}_utils)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
