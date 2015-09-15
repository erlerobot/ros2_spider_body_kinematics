cmake_minimum_required(VERSION 2.8.3)
project(spider_body_kinematics)

#if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-narrowing -Wall -Wdeprecated-declarations")
#  set(CEF_C_COMPILER_FLAGS  -Wno-narrowing)
#endif()

#anita
set(ROS_BUILD_TYPE Debug)
#anita

##############
## Packages ##
##############

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(orocos_kdl)
#find_package(catkin REQUIRED COMPONENTS
#  kdl_parser
#  roscpp
#  rospy
#  std_msgs
#)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)#ROS Middleware (rmw) Interface
find_package(rmw_implementation REQUIRED)#CMake infrastructure and dependencies for rmw implementations

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spider_rpy_kinematics
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS message_runtime
#)

###################################
## ament specific configuration ##
###################################

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

set(target_dependencies
  "rclcpp"
  "std_msgs"
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
##add_executable(rpy_body_kinematics src/rpy_body_kinematics.cpp)
##target_link_libraries(rpy_body_kinematics ${catkin_LIBRARIES})

#add_executable(body_kinematics src/body_kinematics.cpp)

add_executable_for_each_rmw_implementations(body_kinematics src/body_kinematics.cpp
	TARGET_DEPENDENCIES ${target_dependencies} INSTALL)

#target_link_libraries(body_kinematics ${catkin_LIBRARIES})

target_link_libraries(body_kinematics ${ament_LIBRARIES})

#install(
#	TARGETS body_kinematics
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

