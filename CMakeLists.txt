cmake_minimum_required(VERSION 2.8.3)
project(mqtt_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbridge_library
)

catkin_python_setup()

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/mqtt_bridge_node.py.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS
)

#############
## Testing ##
#############

catkin_add_nosetests(test)
