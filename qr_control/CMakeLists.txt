cmake_minimum_required(VERSION 2.8.3)
project(qr_control)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

find_package(catkin REQUIRED COMPONENTS cmake_modules urdf kdl_parser mii_foundation qr_driver)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(orocos_kdl REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qr_control_lib
  CATKIN_DEPENDS kdl_parser
  DEPENDS orocos_kdl
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set(DDP_FILES
  src/test_4_qr_walker.cpp
  src/qr_walk_scheduler.cpp
  src/position_joint_group_controller.cpp
  src/position_reset.cpp
  src/foot_contact.cpp
  src/swing.cpp
  src/math.cpp
  src/gesture_control.cpp)
  
add_library(${PROJECT_NAME}_lib ${DDP_FILES} )


 target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  glog
 )

add_executable(gesture src/uart_test.cpp src/JY901.cpp)
target_link_libraries(gesture
  ${catkin_LIBRARIES}  
 )
