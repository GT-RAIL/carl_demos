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
  message_generation
  rail_manipulation_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ObtainObject.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

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
add_executable(specific_actions src/specific_actions.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(carl_speed_speak
  ${catkin_LIBRARIES}
  espeak
)
target_link_libraries(sequential_tasks ${catkin_LIBRARIES})
target_link_libraries(specific_actions ${catkin_LIBRARIES})

add_dependencies(sequential_tasks carl_dynamixel_gencpp carl_moveit_gencpp carl_navigation_gencpp carl_demos_gencpp)
add_dependencies(specific_actions carl_moveit_gencpp carl_demos_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS carl_speed_speak sequential_tasks specific_actions
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Copy rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
