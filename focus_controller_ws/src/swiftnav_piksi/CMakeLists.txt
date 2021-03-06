cmake_minimum_required(VERSION 2.8.3)
project(swiftnav_piksi)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	diagnostic_updater 
	roscpp 
	sensor_msgs 
	std_srvs 
	std_msgs 
	message_generation 
	rospy
)

## System dependencies are found with CMake's conventions
#find_package(swiftnav REQUIRED)
find_library(SBP sbp)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   loc.msg
   fil.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  swiftnav_piksi
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES piksi piksi_driver
   CATKIN_DEPENDS diagnostic_updater roscpp sensor_msgs std_srvs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(piksi
  src/piksi.c
)
add_library(piksi_driver
  src/piksi_driver.cpp
)


## Declare a cpp executable
add_executable(piksi_node src/piksi_node.cpp)
add_executable(piksitest src/piksi.c)
#add_executable(location src/location.cpp)
#target_link_libraries(location ${catkin_LIBRARIES})

## Add dependencies to the executable
# add_dependencies(yei_piksi_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(piksi
  ${SBP}
)
target_link_libraries(piksitest
  ${SBP}
)
target_link_libraries(piksi_driver
  sbp
  piksi
)
target_link_libraries(piksi_node
  piksi_driver
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS piksi piksi_driver piksi_node piksitest
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yei_piksi.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#catkin_install_python(PROGRAMS src/kalman.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

