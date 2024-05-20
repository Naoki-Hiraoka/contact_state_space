# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  )

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

# find_package(choreonoid)を2回呼ぶとadd_library cannot create imported target "Choreonoid::Cnoid*" because another target with the same name already exists.エラーになるので、親のCMakeLists.txtで1回だけ呼ぶ.
# find_package(choreonoid REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

add_library(TactileOdometry SHARED
  TactileOdometry.cpp
  TactileOdometryService_impl.cpp
  )
target_link_libraries(TactileOdometry
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  )
set_target_properties(TactileOdometry PROPERTIES PREFIX "") # libTactileOdometry.so -> TactileOdometry.so
add_dependencies(TactileOdometry RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(TactileOdometryComp TactileOdometryComp.cpp)
target_link_libraries(TactileOdometryComp TactileOdometry)

install(TARGETS TactileOdometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS TactileOdometryComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
