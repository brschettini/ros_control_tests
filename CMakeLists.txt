cmake_minimum_required(VERSION 2.8.3)
project(ros_control_tests)
set(CMAKE_CXX_STANDARD 11)

set(CATKIN_DEPS 
  hardware_interface
  controller_manager
  default_robot_hw_base
  combined_robot_hw
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPS}
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES dummy_default_robot_hw
 CATKIN_DEPENDS ${CATKIN_DEPS}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(dummy_default_robot_hw src/dummy_default_robot_hw.cpp)
target_link_libraries(dummy_default_robot_hw ${catkin_LIBRARIES})

add_executable(dummy_control_loop src/dummy_control_loop.cpp)
target_link_libraries(dummy_control_loop ${catkin_LIBRARIES} dummy_default_robot_hw)

add_executable(combined_control_loop src/combined_control_loop.cpp)
target_link_libraries(combined_control_loop ${catkin_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ros_control_tests_node.cpp)

add_executable(robot_test src/robot_example1.cpp)
target_link_libraries(robot_test ${catkin_LIBRARIES})

add_executable(robot_test_2 src/robot_example2.cpp)
target_link_libraries(robot_test_2 ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS dummy_default_robot_hw dummy_control_loop combined_control_loop robot_test robot_test_2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_control_tests.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
