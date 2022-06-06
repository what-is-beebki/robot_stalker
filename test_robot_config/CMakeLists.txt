cmake_minimum_required(VERSION 3.0.4)
project(test_robot_config)

find_package(catkin REQUIRED)
#find_package(catkin REQUIRED hector_sensors_description)

catkin_package()

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOTHER_STUFF=1" )
catkin_install_python(PROGRAMS scripts/reach_the_point.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
