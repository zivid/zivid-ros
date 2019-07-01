cmake_minimum_required(VERSION 3.5)
project(zivid_samples)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  zivid_camera
  dynamic_reconfigure
)

catkin_package(
  CATKIN_DEPENDS zivid_camera
)

################
## C++ Sample ##
################

set(SAMPLE_CPP_NAME "sample_capture_cpp")
add_executable(${SAMPLE_CPP_NAME} src/sample_capture.cpp)
add_dependencies(${SAMPLE_CPP_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${SAMPLE_CPP_NAME}
  SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${SAMPLE_CPP_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${SAMPLE_CPP_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###################
## Python Sample ##
###################

install(PROGRAMS
  scripts/sample_capture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

####################
## Launch scripts ##
####################

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
