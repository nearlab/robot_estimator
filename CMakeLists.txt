cmake_minimum_required(VERSION 3.0)
project(robot_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  roscpp
)

#find_package(Eigen3 3.3 REQUIRED NO_MODULE)
set(EIGEN3_INCLUDE_DIRS /usr/share/eigen)

add_message_files(DIRECTORY ${PROJECT_SOURCE_DIR}/msg FILES
  Control.msg
  MarkersParsed.msg
  State.msg
  StateCov.msg
  SpacecraftState.msg
  Imu.msg
  Marker.msg
  Markers.msg
)

generate_messages( DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  roscpp
  message_runtime
)

#########
# BUILD #
#########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(dynamics_node
               src/dynamics_node.cpp)
add_executable(sc_controller_node
               src/sc_controller_node.cpp)
add_executable(markers_node
               src/markers_node.cpp)
add_executable(estimator_node
               src/estimator_node.cpp
               src/simulator.cpp
               src/quatMath.cpp
               src/params.cpp
               src/estimator.cpp)
add_executable(imu_sim_node
               src/quatMath.cpp
               src/params.cpp
               src/imu_sim_node.cpp)

set(EXEC_LIST dynamics_node sc_controller_node markers_node estimator_node imu_sim_node)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${catkin_LIBRARIES})
  add_dependencies(${EXEC_FILE} ${catkin_iEXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
