cmake_minimum_required(VERSION 2.8.3)
project(controller_helene)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  message_generation
  trajectory_msgs
  sensor_msgs
)
 add_message_files(
   FILES
   jointPosition.msg
 )
generate_messages(
   DEPENDENCIES
   std_msgs
 )
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    roscpp
    trajectory_msgs
    sensor_msgs
    std_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_robot_hw_vel src/my_robot_vel.cpp)
add_dependencies(my_robot_hw_vel
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(my_robot_hw_vel
  ${catkin_LIBRARIES}
)
