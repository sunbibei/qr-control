cmake_minimum_required(VERSION 2.8.3)
project(qr_next_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules
  # urdf
  qr_driver
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include

  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS qr_driver
  DEPENDS
)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(SRC_ROOT          src)
set(SRC_GAITS_ROOT    ${SRC_ROOT}/gait)
set(SRC_ROBOT_ROOT    ${SRC_ROOT}/robot)
set(SRC_ORIGIN_ROOT   ${SRC_ROBOT_ROOT}/origin)
set(SRC_ADT_ROOT      ${SRC_ROOT}/adt)

set(GAIT_FILES        ${SRC_GAITS_ROOT}/gait_manager.cpp
                      ${SRC_GAITS_ROOT}/gait_base.cpp
                      ${SRC_GAITS_ROOT}/creep/creep.cpp
)

set(ROBOT_FILES       ${SRC_ORIGIN_ROOT}/origin.cpp
                      ${SRC_ORIGIN_ROOT}/origin_test.cpp

                      ${SRC_ROBOT_ROOT}/data_leg.cpp
                      ${SRC_ROBOT_ROOT}/math_leg.cpp
                      ${SRC_ROBOT_ROOT}/robot_leg.cpp
                      ${SRC_ROBOT_ROOT}/robot_body.cpp
)

set(BIN_FILES         ${SRC_ROOT}/main.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}_repository
  ${GAIT_FILES}
  ${ROBOT_FILES}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_test
  ${BIN_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_repository
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  qr_driver_foundation
)

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}_repository
)




set(DDP_FILES
  src/qr_control/position_joint_group_controller.cpp
  src/qr_control/foot_contact.cpp
  src/qr_control/swing.cpp
  src/qr_control/math.cpp
  src/qr_control/gesture_control.cpp)
  
add_library(${PROJECT_NAME}_lib ${DDP_FILES} )

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  glog
)
