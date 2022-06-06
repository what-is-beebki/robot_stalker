cmake_minimum_required(VERSION 3.0.4)
project(test_robot_description)

find_package(catkin REQUIRED)
#find_package(catkin REQUIRED hector_sensors_description)

catkin_package()

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOTHER_STUFF=1" )
