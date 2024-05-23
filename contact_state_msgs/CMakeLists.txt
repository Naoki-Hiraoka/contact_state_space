cmake_minimum_required(VERSION 3.0.2)
project(contact_state_msgs)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  geometry_msgs
  )

# ## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Contact.msg
  ContactArray.msg
  )

# initialize rtmbuild (generate_messages is called)
rtmbuild2_init(rtmbuild2 geometry_msgs) # ここにidlとmsgが依存する外部パッケージを書く

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ContactStateSkel ContactStateStub
  DEPENDS openrtm_aist
  )

###########
## Build ##
###########

# generate idl
rtmbuild2_genidl()

# generate bridge
# rtmbuild2_genbridge()

add_custom_target(genidl ALL DEPENDS RTMBUILD2_${PROJECT_NAME}_genrpc)
