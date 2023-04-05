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

set(THIS_PACKAGE_DEPS dynamic_reconfigure filters geometric_shapes laser_geometry moveit_core moveit_ros_perception roscpp sensor_msgs tf2 tf2_ros urdf visualization_msgs)
set(MESSAGE_DEPS geometry_msgs std_msgs)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_DEPS} ${MESSAGE_DEPS} message_generation pcl_conversions tf2_eigen tf2_sensor_msgs)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
# find *absolute* paths to LIBFCL_* paths
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
set(LIBFCL_LIBRARIES)
foreach(_lib ${LIBFCL_PC_LIBRARIES})
  find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
endforeach()


add_message_files(FILES OrientedBoundingBox.msg OrientedBoundingBoxStamped.msg Sphere.msg SphereStamped.msg)
generate_messages(DEPENDENCIES ${MESSAGE_DEPS})

catkin_package(
    CATKIN_DEPENDS ${THIS_PACKAGE_DEPS} ${MESSAGE_DEPS} message_runtime
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} RayCastingShapeMask TFFramesWatchdog ${PROJECT_NAME}_utils
)

include_directories(SYSTEM ${LIBFCL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

set(UTILS_SRCS
  src/utils/bodies.cpp
  src/utils/cloud.cpp
  src/utils/shapes.cpp
  src/utils/string_utils.cpp
  src/utils/tf2_eigen.cpp
  src/utils/tf2_sensor_msgs.cpp
  src/utils/time_utils.cpp)

if (PCL_VERSION VERSION_LESS "1.10")
  set(UTILS_SRCS ${UTILS_SRCS} src/utils/crop_box.cpp)
endif()

add_library(${PROJECT_NAME}_utils ${UTILS_SRCS})
target_link_libraries(${PROJECT_NAME}_utils ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES} ${PCL_LIBRARIES})

add_library(tf2_sensor_msgs_rbf src/utils/tf2_sensor_msgs.cpp)
target_link_libraries(tf2_sensor_msgs_rbf PRIVATE ${PROJECT_NAME}_utils PUBLIC ${catkin_LIBRARIES})
add_dependencies(tf2_sensor_msgs_rbf ${catkin_EXPORTED_TARGETS})

add_library(RayCastingShapeMask src/RayCastingShapeMask.cpp)
target_link_libraries(RayCastingShapeMask ${PROJECT_NAME}_utils ${catkin_LIBRARIES})
add_dependencies(RayCastingShapeMask ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(TFFramesWatchdog src/TFFramesWatchdog.cpp)
target_link_libraries(TFFramesWatchdog ${PROJECT_NAME}_utils ${catkin_LIBRARIES})
add_dependencies(TFFramesWatchdog ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(${PROJECT_NAME} src/RobotBodyFilter.cpp)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_utils
  tf2_sensor_msgs_rbf
  TFFramesWatchdog
  RayCastingShapeMask
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} TFFramesWatchdog RayCastingShapeMask ${PROJECT_NAME}_utils tf2_sensor_msgs_rbf
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES laser_filters.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)

install(FILES rviz/debug.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_set_utils test/test_set_utils.cpp)
  target_link_libraries(test_set_utils ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_shapes_rbf test/test_shapes.cpp)
  target_link_libraries(test_shapes_rbf ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_urdf_eigen test/test_urdf_eigen.cpp)
  target_link_libraries(test_urdf_eigen ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_time_utils test/test_time_utils.cpp)
  target_link_libraries(test_time_utils ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_tf2_eigen test/test_tf2_eigen.cpp)
  target_link_libraries(test_tf2_eigen ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_string_utils test/test_string_utils.cpp)
  target_link_libraries(test_string_utils ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_bodies test/test_bodies.cpp)
  target_link_libraries(test_bodies ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_cloud test/test_cloud.cpp)
  target_link_libraries(test_cloud ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_tf2_sensor_msgs test/test_tf2_sensor_msgs.cpp)
  target_link_libraries(test_tf2_sensor_msgs tf2_sensor_msgs_rbf ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  catkin_add_gtest(test_xmlrpc_traits test/test_xmlrpc_traits.cpp)
  target_link_libraries(test_xmlrpc_traits ${PROJECT_NAME}_utils ${catkin_LIBRARIES})

  add_rostest_gtest(test_filter_utils test/test_filter_utils.test test/test_filter_utils.cpp)
  target_link_libraries(test_filter_utils ${PROJECT_NAME} ${catkin_LIBRARIES})

  catkin_add_gtest(test_tf_frames_watchdog test/test_tf_frames_watchdog.cpp)
  target_link_libraries(test_tf_frames_watchdog TFFramesWatchdog ${catkin_LIBRARIES})

  catkin_add_gtest(test_ray_casting_shape_mask test/test_ray_casting_shape_mask.cpp)
  target_link_libraries(test_ray_casting_shape_mask RayCastingShapeMask ${catkin_LIBRARIES})
  if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=1)
  else()
    target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=0)
  endif()

  add_rostest_gtest(test_robot_body_filter test/test_robot_body_filter.test test/test_robot_body_filter.cpp)
  target_link_libraries(test_robot_body_filter ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_compile_options(test_robot_body_filter PRIVATE -fno-var-tracking-assignments)  # speed up build
endif()