cmake_minimum_required(VERSION 2.8.3)
project(ik_solver_barrett_wam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  actionlib_msgs
  move_base_msgs
  tf2_msgs
  tf
# turtlebot3_msgs
  message_generation
  dynamic_reconfigure
)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#add_message_files(
#  FILES
#  PadOrder.msg
#  AvailableItemList.msg
#  ServiceStatus.msg)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  InvKineWam.srv
#  PoseMove.srv
#  CartPosMove.srv
#  OrtnMove.srv
#  JointMove.srv
#  BHandFingerVel.srv
#  BHandGraspVel.srv
#  BHandGraspPos.srv
#  BHandSpreadPos.srv
#  BHandSpreadVel.srv
#  Hold.srv
#  GravityComp.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  #std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS
   roscpp
   std_msgs
   geometry_msgs
   tf2_msgs
#   turtlebot3_msgs
   message_runtime
   DEPENDS system_lib
   actionlib_msgs
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
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/uwarl_tasks.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/uwarl_tasks_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# For actual robot
add_executable(ik_solver_wam src/main.cpp src/InverseKinematicsWAM.cpp src/InverseKinematicsWAM_initialize.cpp src/InverseKinematicsWAM_terminate.cpp src/InverseKinematicsWAM_rtwutil.cpp src/acos.cpp src/cross.cpp src/norm.cpp src/quat2rotm.cpp src/rotm2quat.cpp src/rtGetInf.cpp src/rtGetNaN.cpp src/rt_nonfinite.cpp src/schur.cpp src/sqrt.cpp src/xdhseqr.cpp src/xdlanv2.cpp src/xgehrd.cpp src/xgemv.cpp src/xgerc.cpp src/xnrm2.cpp src/xrot.cpp src/xzggev.cpp src/xzhgeqz.cpp src/xzlarf.cpp src/xzlarfg.cpp src/xzlartg.cpp src/xzlascl.cpp src/xztgevc.cpp)
add_dependencies(ik_solver_wam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ik_solver_wam ${catkin_LIBRARIES})

add_executable(ik_solver_wam_repeatable src/main_repeatable.cpp src/InverseKinematicsWAM.cpp src/InverseKinematicsWAM_initialize.cpp src/InverseKinematicsWAM_terminate.cpp src/InverseKinematicsWAM_rtwutil.cpp src/acos.cpp src/cross.cpp src/norm.cpp src/quat2rotm.cpp src/rotm2quat.cpp src/rtGetInf.cpp src/rtGetNaN.cpp src/rt_nonfinite.cpp src/schur.cpp src/sqrt.cpp src/xdhseqr.cpp src/xdlanv2.cpp src/xgehrd.cpp src/xgemv.cpp src/xgerc.cpp src/xnrm2.cpp src/xrot.cpp src/xzggev.cpp src/xzhgeqz.cpp src/xzlarf.cpp src/xzlarfg.cpp src/xzlartg.cpp src/xzlascl.cpp src/xztgevc.cpp)
add_dependencies(ik_solver_wam_repeatable ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ik_solver_wam_repeatable ${catkin_LIBRARIES})

# add_executable(pad_virtual src/pad_virtual.cpp)
# add_dependencies(pad_virtual ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(pad_virtual ${catkin_LIBRARIES})

#############
## Install ##
#############


# install(TARGETS service_core pad_virtual pose_initialization #call_initial_pose
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY launch param 
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

