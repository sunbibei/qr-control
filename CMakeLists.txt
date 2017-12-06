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
  LIBRARIES
  CATKIN_DEPENDS qr_driver
  DEPENDS
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(SRC_ROOT          src)
set(SRC_GAITS_ROOT    ${SRC_ROOT}/gait)
set(SRC_ROBOT_ROOT    ${SRC_ROOT}/robot)
set(SRC_ADT_ROOT      ${SRC_ROOT}/adt)

set(GAIT_FILES        ${SRC_GAITS_ROOT}/gait_manager.cpp
                      ${SRC_GAITS_ROOT}/gait_base.cpp
                      ${SRC_GAITS_ROOT}/creep/creep.cpp
)

set(ROBOT_FILES       ${SRC_ROBOT_ROOT}/data_leg.cpp
                      ${SRC_ROBOT_ROOT}/math_leg.cpp
                      ${SRC_ROBOT_ROOT}/robot_leg.cpp
                      ${SRC_ROBOT_ROOT}/robot_body.cpp
)

set(CTRL_FILES        ${SRC_ROOT}/mii_control.cpp
)

set(TEST_FILES        ${SRC_ROOT}/main.cpp
)

## Declare a C++ library
add_library(${PROJECT_NAME}_repository
  ${GAIT_FILES}
  ${ROBOT_FILES}
)

add_library(${PROJECT_NAME}_framwork
  ${CTRL_FILES}
)

## Declare a C++ executable
#[[add_executable(${PROJECT_NAME}_test
  ${BIN_FILES}
)]]

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_repository
  ${catkin_LIBRARIES}
  qr_driver_foundation
  qr_driver_registry
)

target_link_libraries(${PROJECT_NAME}_framwork
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_repository
)

#[[target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}_repository
)]]

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_repository
                ${PROJECT_NAME}_framwork
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir include launch config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}_test
  ${TEST_FILES}
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}_repository
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)