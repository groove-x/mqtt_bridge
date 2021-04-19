cmake_minimum_required(VERSION 3.0.2)
project(mqtt_bridge)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()


install(PROGRAMS
  scripts/mqtt_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
