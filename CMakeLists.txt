cmake_minimum_required(VERSION 2.8.3)
project(intel_aero_rtf_gr871)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  cv_bridge_new
  image_transport
  tf
  message_generation
  cmake_modules
)

find_package(Eigen REQUIRED) 

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# don't use opencv as found by any prior running of find_package
unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)

set(TMP_PREFIX_PATH ${CMAKE_PREFIX_PATH})
set(CMAKE_PREFIX_PATH "/usr/local")
find_package(OpenCV 3.1.0 REQUIRED)

# restore CMAKE_PREFIX_PATH and other cached variables
# so nothing other package finds this opencv
set(CMAKE_PREFIX_PATH ${TMP_PREFIX_PATH})

unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)




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
   controllerLog.msg
   vicon.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   position.srv
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
   std_msgs
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
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime DEPENDS Eigen)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}    
  src/observers/ekf
  src/FastSLAM
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_subdirectory(src/observers)
add_subdirectory(src/FastSLAM)
set(HEADER_FILES include/utils.h)
add_library(utils src/utils.cpp ${HEADER_FILES})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/uav_offboard.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(offb_node src/offb_node.cpp)
add_executable(true_position src/true_position.cpp)

add_executable(depthgrab src/depthgrab.cpp)
add_executable(markers src/markers.cpp)
add_executable(test_imagepub src/test_imagepub.cpp)
add_executable(test_imagesub src/test_imagesub.cpp)

add_executable(controller src/controller.cpp ${HEADER_FILES})

add_executable(Mtest src/Mtest.cpp ${FASTSLAM_HEADER_FILES}) 
add_executable(FastSLAM_node src/FastSLAM_node.cpp ${FASTSLAM_HEADER_FILES} ${HEADER_FILES})




## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")



## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
add_dependencies(true_position ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})

add_dependencies(depthgrab ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
add_dependencies(markers ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
add_dependencies(test_imagepub ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
add_dependencies(test_imagesub ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})

add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS} intel_aero_rtf_gr871_generate_messages_cpp)

add_dependencies(Mtest ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS}) 
add_dependencies(FastSLAM_node ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
target_link_libraries(offb_node ${catkin_LIBRARIES})
target_link_libraries(true_position ${catkin_LIBRARIES})

target_link_libraries(depthgrab ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(markers ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(test_imagepub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(test_imagesub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

target_link_libraries(controller ${catkin_LIBRARIES} ekf utils)
#target_link_libraries(controller ${catkin_LIBRARIES})

target_link_libraries(Mtest ${catkin_LIBRARIES} ${Eigen_LIBRARIES} FastSLAM) 
target_link_libraries(FastSLAM_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} FastSLAM utils)



## Build vicon_j package
add_executable(vicon_pix src/vicon_j.cpp src/udp.cpp src/vicon.cpp)
add_dependencies(vicon_pix intel_aero_rtf_gr871_generate_messages_cpp)
target_link_libraries(vicon_pix ${catkin_LIBRARIES})




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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uav_offboard.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
