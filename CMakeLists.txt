cmake_minimum_required(VERSION 2.8.3)
project(qr_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules urdf kdl_parser)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(orocos_kdl REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qr_control_lib
  CATKIN_DEPENDS kdl_parser
  #DEPENDS system_lib glog orocos_kdl
)


include_directories(include ${catkin_INCLUDE_DIRS})

set(DDP_FILES
  src/position_joint_group_controller.cpp
  src/foot_contact.cpp
  src/swing.cpp
  src/math.cpp
  src/gesture_control.cpp)
  
add_library(qr_control_lib ${DDP_FILES} )


 target_link_libraries(qr_control_lib
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  glog
 )
