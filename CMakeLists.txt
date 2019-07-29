cmake_minimum_required(VERSION 2.8.3)
project(robot_body_filter)

set(CMAKE_CXX_STANDARD 17)  # we're using std::optional
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(THIS_PACKAGE_DEPS dynamic_reconfigure filters geometric_shapes laser_geometry moveit_core moveit_ros_perception roscpp sensor_msgs tf2 tf2_ros urdf visualization_msgs)
set(MESSAGE_DEPS geometry_msgs std_msgs)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_DEPS} ${MESSAGE_DEPS} message_generation tf2_eigen tf2_sensor_msgs)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
find_library(LIBFCL_LIBRARIES fcl HINTS ${LIBFCL_PC_LIBRARY_DIRS})


add_message_files(FILES OrientedBoundingBox.msg OrientedBoundingBoxStamped.msg Sphere.msg SphereStamped.msg)
generate_messages(DEPENDENCIES ${MESSAGE_DEPS})

catkin_package(
    CATKIN_DEPENDS ${THIS_PACKAGE_DEPS} ${MESSAGE_DEPS} message_runtime
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_utils
)

include_directories(SYSTEM ${LIBFCL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_utils
  src/utils/bodies.cpp
  src/utils/cloud.cpp
  src/utils/obb.cpp
  src/utils/shapes.cpp
  src/utils/string_utils.cpp
  src/utils/tf2_eigen.cpp
  src/utils/tf2_sensor_msgs.cpp
  src/utils/time_utils.cpp
)
target_link_libraries(${PROJECT_NAME}_utils ${catkin_LIBRARIES} ${LIBFCL_LIBRARIES})

add_library(${PROJECT_NAME}
  src/RobotBodyFilter.cpp
  src/TFFramesWatchdog.cpp
  src/RayCastingShapeMask.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_utils
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES laser_filters.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)

install(FILES rviz/debug.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
