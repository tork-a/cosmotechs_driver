cmake_minimum_required(VERSION 2.8.3)
project(cosmotechs_driver)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  actionlib_msgs
  message_generation
)

## Uncomment this if the package has a setup.py.
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(DIRECTORY srv)
add_action_files(DIRECTORY action)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(INCLUDE_DIRS include)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pcio32ha_node src/pcio32ha_node.cpp)
add_executable(pcpg23i_node src/pcpg23i_node.cpp)

## Rename C++ executable without prefix
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(pcio32ha_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pcpg23i_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pcio32ha_node ${catkin_LIBRARIES})
target_link_libraries(pcpg23i_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS pcio32ha_node pcpg23i_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/cosmotechs.test)
endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cosmotechs_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
