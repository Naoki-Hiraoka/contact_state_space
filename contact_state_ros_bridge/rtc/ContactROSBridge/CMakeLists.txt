add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp
  geometry_msgs
  tf2
  contact_state_msgs
  eigen_rtm_conversions
  eigen_conversions)
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS})
add_library(ContactROSBridge SHARED
  ContactROSBridge.cpp)
target_link_libraries(ContactROSBridge
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES})
set_target_properties(ContactROSBridge PROPERTIES PREFIX "") # libContactROSBridge.so -> ContactROSBridge.so
#add_dependencies(ContactROSBridge RTMBUILD3_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(ContactROSBridgeComp ContactROSBridgeComp.cpp)
target_link_libraries(ContactROSBridgeComp ContactROSBridge)
