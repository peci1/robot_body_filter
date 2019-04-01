cmake_minimum_required(VERSION 2.8.3)
project(robot_self_filter)

find_package(catkin REQUIRED COMPONENTS filters geometric_shapes resource_retriever roscpp sensor_msgs tf tf_conversions urdf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS signals)

catkin_package(
    CATKIN_DEPENDS filters geometric_shapes resource_retriever roscpp sensor_msgs tf urdf visualization_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/self_mask.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_filter src/test_filter.cpp)
target_link_libraries(test_filter ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(self_filter src/self_filter.cpp)
target_link_libraries(self_filter ${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} test_filter self_filter
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
