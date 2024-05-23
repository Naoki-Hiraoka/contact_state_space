cmake_minimum_required(VERSION 3.0.2)
project(contact_state_ros_bridge)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  tf
  tf_conversions
  eigen_conversions
  contact_state_msgs
  )

# initialize rtmbuild (generate_messages is called)
rtmbuild2_init()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES contact_state_ros_bridge
#  CATKIN_DEPENDS openrtm_aist
#  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
 )

add_subdirectory(rtc)
