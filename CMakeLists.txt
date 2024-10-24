cmake_minimum_required(VERSION 3.1.2)
project(leader_follower_task_tutorial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nav_msgs
  std_msgs
  mrs_lib
  dynamic_reconfigure
  uvdar_core
  )

generate_dynamic_reconfigure_options(
  cfg/Follower.cfg
)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp roslib std_msgs nav_msgs mrs_lib uvdar_core
  LIBRARIES follower
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_library(follower src/follower.cpp src/velocity_estimator.cpp)
add_dependencies(follower
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )
target_link_libraries(follower
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_executable(supervisor src/supervisor.cpp src/message_utils.cpp)
add_dependencies(supervisor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(supervisor ${catkin_LIBRARIES} follower)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


