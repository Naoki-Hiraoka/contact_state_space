cmake_minimum_required(VERSION 3.0.2)
project(contact_state_rtc)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  )

# find_package(choreonoid)を2回呼ぶとadd_library cannot create imported target "Choreonoid::Cnoid*" because another target with the same name already exists.エラーになるので、ここのCMakeLists.txtで1回だけ呼ぶ.
find_package(choreonoid REQUIRED)

# initialize rtmbuild (generate_messages is called)
rtmbuild2_init(rtmbuild2) # ここにidlとmsgが依存する外部パッケージを書く

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES TactileOdometrySeviceSkel TactileOdometrySeviceStub
  DEPENDS openrtm_aist
  )

###########
## Build ##
###########

# generate idl
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

add_subdirectory(rtc)
