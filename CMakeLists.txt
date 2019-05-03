cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_tunnel_nav)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  gazebo_ros
  gazebo_plugins
  hector_gazebo_plugins
  hector_quadrotor_description
  hector_quadrotor_gazebo
  hector_gazebo_worlds
  hector_uav_msgs
  octomap_ros
  octomap_msgs
  roscpp
  rospy
  std_msgs
  message_generation

  laser_geometry
  nav_msgs
  mrpt_bridge
  visualization_msgs
  dynamic_reconfigure
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(gazebo REQUIRED)
find_package(ignition-transport4 REQUIRED)

# Octomap
find_package(octomap REQUIRED)

# MRPT packages
find_package(MRPT 1.5 REQUIRED gui obs slam)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
if(MRPT_VERSION VERSION_LESS "1.9.9")
  set (CMAKE_CXX_STANDARD 14)
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
add_message_files(
  FILES
  Com.msg
#   Message1.msg
#   Message2.msg
)

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
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quadrotor_tunnel_nav
  CATKIN_DEPENDS
    message_runtime gazebo_ros hector_gazebo_plugins hector_quadrotor_description hector_gazebo_worlds roscpp rospy std_msgs
    visualization_msgs nav_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
  INCLUDE_DIRS
  LIBRARIES
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)
# Create a static library with the Protobuf messages used internally.
set(protobuf_lib_name QuadrotorTunnelNavProtobuf)
add_subdirectory(src/adhoc/protobuf)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS}
                            PROPERTIES GENERATED TRUE)
add_library(${protobuf_lib_name} STATIC ${PROTO_SOURCES})
add_dependencies(${protobuf_lib_name} protobuf_compilation)
set_target_properties(${protobuf_lib_name}
  PROPERTIES POSITION_INDEPENDENT_CODE ON COMPILE_FLAGS "-Wno-unused-parameter")

# Create the libAdHocNetPlugin.so library.
set(adhocnet_plugin_name AdHocNetPlugin)
add_library(${adhocnet_plugin_name} src/adhoc/AdHocNetPlugin.cc)
target_include_directories(${adhocnet_plugin_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_include_directories(${adhocnet_plugin_name}
  PRIVATE src/adhoc)
target_link_libraries(${adhocnet_plugin_name}
  ${catkin_LIBRARIES}
  ${protobuf_lib_name}
  ${ignition-transport4_LIBRARIES}
)
add_dependencies(${adhocnet_plugin_name}
  protobuf_compilation
)

# Create the libAdHocClientPlugin.so library.
set(adhocclient_plugin_name AdHocClientPlugin)
add_library(${adhocclient_plugin_name} src/adhoc/AdHocClientPlugin.cc)
target_include_directories(${adhocclient_plugin_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_include_directories(${adhocclient_plugin_name}
  PRIVATE src/adhoc)
target_link_libraries(${adhocclient_plugin_name}
  ${catkin_LIBRARIES}
  ${protobuf_lib_name}
  ${ignition-transport4_LIBRARIES}
  ssl crypto
)
add_dependencies(${adhocclient_plugin_name}
  protobuf_compilation
)

## Declare a C++ library
# add_library(quadrotor_tunnel_nav
#   src/${PROJECT_NAME}/quadrotor_tunnel_nav.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(quadrotor_tunnel_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(quadrotor_tunnel_nav_node src/quadrotor_tunnel_nav_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(quadrotor_tunnel_nav_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(quadrotor_tunnel_nav_node
#   ${catkin_LIBRARIES}
# )

add_executable(main_control_node src/reactive/mains/main_control.cpp)
add_dependencies(main_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_control_node ${catkin_LIBRARIES})

add_library(obs_avoid_lib src/reactive/Obs_Avoid.cpp)
add_dependencies(obs_avoid_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obs_avoid_lib ${catkin_LIBRARIES})
add_executable(obs_avoid_node src/reactive/mains/obstacle_avoidance.cpp)
add_dependencies(obs_avoid_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} obs_avoid_lib)
target_link_libraries(obs_avoid_node ${catkin_LIBRARIES} obs_avoid_lib)

add_library(altitude_lib src/reactive/Keep_Alt.cpp)
add_dependencies(altitude_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(altitude_lib ${catkin_LIBRARIES})
add_executable(altitude_node src/reactive/mains/altitude.cpp)
add_dependencies(altitude_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} altitude_lib)
target_link_libraries(altitude_node ${catkin_LIBRARIES} altitude_lib)

add_library(turn_lib src/reactive/Turn.cpp)
add_dependencies(turn_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turn_lib ${catkin_LIBRARIES})
add_executable(turn_node src/reactive/mains/turn.cpp)
add_dependencies(turn_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} turn_lib)
target_link_libraries(turn_node ${catkin_LIBRARIES} turn_lib)

add_library(steer_lib src/reactive/Steer.cpp)
add_dependencies(steer_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(steer_lib ${catkin_LIBRARIES})
add_executable(steer_node src/reactive/mains/steer.cpp)
add_dependencies(steer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} steer_lib)
target_link_libraries(steer_node ${catkin_LIBRARIES} steer_lib)

add_library(middle_line_lib src/reactive/Middle_Line.cpp)
add_dependencies(middle_line_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(middle_line_lib ${catkin_LIBRARIES})
add_executable(middle_line_node src/reactive/mains/middle_line.cpp)
add_dependencies(middle_line_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} middle_line_lib)
target_link_libraries(middle_line_node ${catkin_LIBRARIES} middle_line_lib)

add_library(go_straight_lib src/reactive/Go_Straight.cpp)
add_dependencies(go_straight_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_straight_lib ${catkin_LIBRARIES})
add_executable(go_straight_node src/reactive/mains/go_straight.cpp)
add_dependencies(go_straight_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} go_straight_lib)
target_link_libraries(go_straight_node ${catkin_LIBRARIES} go_straight_lib)


# Replacements
add_library(right_wall_lib src/wall_follow/Right_Wall.cpp)
add_dependencies(right_wall_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} middle_line_lib)
target_link_libraries(right_wall_lib ${catkin_LIBRARIES} middle_line_lib)
add_executable(right_wall_node src/wall_follow/mains/right_wall.cpp)
add_dependencies(right_wall_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} right_wall_lib)
target_link_libraries(right_wall_node ${catkin_LIBRARIES} right_wall_lib)

add_library(keep_going_lib src/wall_follow/Keep_Going.cpp)
add_dependencies(keep_going_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} go_straight_lib)
target_link_libraries(keep_going_lib ${catkin_LIBRARIES} go_straight_lib)
add_executable(keep_going_node src/wall_follow/mains/keep_going.cpp)
add_dependencies(keep_going_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} keep_going_lib)
target_link_libraries(keep_going_node ${catkin_LIBRARIES} keep_going_lib)


# SLAM
add_executable(octomap_only src/octomap_only/octomap.cpp)
add_dependencies(octomap_only ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_only ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

# RBPF
add_library(custom_octomap src/rbpf/CustomOctoMap.cpp)
add_library(slam_core src/rbpf/mrpt_rbpf_slam.cpp)
add_executable(mrpt_rbpf_slam
    src/rbpf/mrpt_rbpf_slam_wrapper.cpp
    src/rbpf/mrpt_rbpf_slam_node.cpp
    src/rbpf/options.cpp
)
target_link_libraries(mrpt_rbpf_slam
    ${MRPT_LIBRARIES}
    ${catkin_LIBRARIES}
    slam_core
    custom_octomap
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS quadrotor_tunnel_nav quadrotor_tunnel_nav_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_quadrotor_tunnel_nav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
