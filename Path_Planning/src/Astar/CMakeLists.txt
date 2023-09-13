cmake_minimum_required(VERSION 3.0.2)
project(pathPlanning)

# Eigen
# include(cmake/FindEigen.cmake)
# include_directories(${EIGEN_INCLUDE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pathPlanning
CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(head_src
  include/Astar.h
  src/Astar.cpp

  # include/matplotlibcpp.h
)


add_executable(run_planning src/run_planning.cpp)


add_dependencies(head_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(run_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(head_src
  ${catkin_LIBRARIES}
)

target_link_libraries(run_planning
  head_src
  ${catkin_LIBRARIES}
)

