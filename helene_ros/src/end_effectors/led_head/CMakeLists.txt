cmake_minimum_required(VERSION 3.0.2)
project(led_head)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    message_generation
    std_msgs
    gazebo_plugins
    gazebo_ros
    roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(gazebo REQUIRED)

option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)
if (ENABLE_PROFILER)
  find_package(ignition-common3 QUIET)
  if (ignition-common3_FOUND)
    add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
    message("Profiler is active")
  else()
    message("Can't find Ignition common3. Profiler will not be actived")
    add_definitions("-DIGN_PROFILER_ENABLE=0" "-DIGN_PROFILER_REMOTERY=0")
  endif()
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    gazebo_plugins
    gazebo_ros
    roscpp
    EIGEN3
)

add_library(led_light_plugin src/led_light_plugin.cpp)
set_target_properties(led_light_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(led_light_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(led_light_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS led_light_plugin DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/)
