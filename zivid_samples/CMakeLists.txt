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

#################
## C++ Samples ##
#################

function(register_cpp_sample)
  cmake_parse_arguments(
    ARG
    ""
    "NAME;SRC"
    ""
    ${ARGN}
  )
  add_executable(${ARG_NAME} ${ARG_SRC})
  add_dependencies(${ARG_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(${ARG_NAME}
    SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${ARG_NAME}
    ${catkin_LIBRARIES}
  )
  install(TARGETS ${ARG_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endfunction()

register_cpp_sample(NAME sample_capture_cpp SRC src/sample_capture.cpp)
register_cpp_sample(NAME sample_capture_and_save_cpp SRC src/sample_capture_and_save.cpp)
register_cpp_sample(NAME sample_capture_2d_cpp SRC src/sample_capture_2d.cpp)
register_cpp_sample(NAME sample_capture_assistant_cpp SRC src/sample_capture_assistant.cpp)
register_cpp_sample(NAME sample_capture_with_settings_from_yml_cpp SRC src/sample_capture_with_settings_from_yml.cpp)

####################
## Python Samples ##
####################

function(register_python_sample)
  cmake_parse_arguments(
    ARG
    ""
    "SRC"
    ""
    ${ARGN}
    )
  install(PROGRAMS
    ${ARG_SRC}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endfunction()

register_python_sample(SRC scripts/sample_capture.py)
register_python_sample(SRC scripts/sample_capture_and_save.py)
register_python_sample(SRC scripts/sample_capture_2d.py)
register_python_sample(SRC scripts/sample_capture_assistant.py)
register_python_sample(SRC scripts/sample_capture_with_settings_from_yml.py)

####################
## Launch scripts ##
####################

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
