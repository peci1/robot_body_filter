cmake_minimum_required(VERSION 2.8.3)
project(robot_body_filter)

set(CMAKE_CXX_STANDARD 17)  # we're using std::optional
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-O3")

set(THIS_PACKAGE_DEPS filters geometric_shapes laser_geometry moveit_core moveit_ros_perception roscpp sensor_msgs tf2 tf2_ros urdf visualization_msgs)
set(MESSAGE_DEPS geometry_msgs std_msgs)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_DEPS} ${MESSAGE_DEPS} message_generation tf2_eigen tf2_sensor_msgs)
find_package(PCL REQUIRED COMPONENTS common filters)

add_message_files(FILES Sphere.msg SphereStamped.msg)
generate_messages(DEPENDENCIES ${MESSAGE_DEPS})

catkin_package(
    CATKIN_DEPENDS ${THIS_PACKAGE_DEPS} ${MESSAGE_DEPS} message_runtime
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} utils
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(utils
  src/utils/bodies.cpp
  src/utils/cloud.cpp
  src/utils/shapes.cpp
  src/utils/string_utils.cpp
  src/utils/tf2_eigen.cpp
  src/utils/time_utils.cpp
)
target_link_libraries(utils ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
  src/RobotBodyFilter.cpp
  src/TFFramesWatchdog.cpp
  src/RayCastingShapeMask.cpp)
target_link_libraries(${PROJECT_NAME} utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} utils
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES laser_filters.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)

install(FILES rviz/debug.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
