cmake_minimum_required(VERSION 3.0.2)
project(project_2dto3d)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
set( LIBRARY_PATH "${PROJECT_SOURCE_DIR}/lib/${ARCHITECTURE}" )
set( GL_LIBRARY "${LIBRARY_PATH}/libobject_detection.so" "${LIBRARY_PATH}/libgl_driver.so" "${LIBRARY_PATH}/libserial.so" )

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_scan src/lidar_scan.cpp)
add_executable(change_finder src/change_finder.cpp)
add_executable(degree_finder src/degree_finder.cpp)
add_executable(point_cluster src/point_cluster.cpp)

target_link_libraries(lidar_scan ${catkin_LIBRARIES} ${GL_LIBRARY})
target_link_libraries(change_finder ${catkin_LIBRARIES} ${GL_LIBRARY})
target_link_libraries(degree_finder ${catkin_LIBRARIES} ${GL_LIBRARY})
target_link_libraries(point_cluster ${catkin_LIBRARIES})

add_dependencies(lidar_scan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GL_LIBRARY})
add_dependencies(change_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GL_LIBRARY})
add_dependencies(degree_finder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${GL_LIBRARY})
add_dependencies(point_cluster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
