cmake_minimum_required(VERSION 2.8.3)
project(carl_demos)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  carl_dynamixel
  carl_moveit
  carl_navigation
  geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(carl_speed_speak src/carl_speed_speak.cpp)
add_executable(sequential_tasks src/sequential_tasks.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(carl_speed_speak
  ${catkin_LIBRARIES}
  espeak
)
target_link_libraries(sequential_tasks ${catkin_LIBRARIES})

add_dependencies(sequential_tasks carl_dynamixel_gencpp carl_moveit_gencpp carl_navigation_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS carl_speed_speak
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
