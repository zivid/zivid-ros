cmake_minimum_required(VERSION 3.5)
project(zivid_camera LANGUAGES CXX)

if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
  add_compile_options(-std=c++17)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(COMPILER_WARNINGS "Turn on warnings and warnings-as-errors during compilation" OFF)
if(COMPILER_WARNINGS)
  include(CompilerWarnings NO_POLICY_SCOPE)
endif()

function(turn_on_compiler_warnings_if_enabled TARGET)
  if(COMPILER_WARNINGS)
    set_target_warning_compile_options(${TARGET})
  endif()
endfunction()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  message_generation
  image_transport
  nodelet
)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Zivid 2.5.0 COMPONENTS Core REQUIRED)
message(STATUS "Found Zivid version ${Zivid_VERSION}")

find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

set(SETTINGS_GENERATOR_TARGET_NAME ${PROJECT_NAME}_settings_generator)
add_executable(${SETTINGS_GENERATOR_TARGET_NAME} src/settings_generator.cpp)
target_include_directories(
  ${SETTINGS_GENERATOR_TARGET_NAME}
  SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${SETTINGS_GENERATOR_TARGET_NAME} PRIVATE Zivid::Core Boost::filesystem)

set(GENERATOR_TARGET_NAME ${PROJECT_NAME}_generator)
add_custom_target(
  ${GENERATOR_TARGET_NAME}
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/state_srv_files
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/generated_headers
  COMMAND ${SETTINGS_GENERATOR_TARGET_NAME}
  COMMAND chmod a+x ${CMAKE_CURRENT_BINARY_DIR}/SettingsAcquisition.cfg
  COMMAND chmod a+x ${CMAKE_CURRENT_BINARY_DIR}/Settings.cfg
  COMMAND chmod a+x ${CMAKE_CURRENT_BINARY_DIR}/Settings2DAcquisition.cfg
  COMMAND chmod a+x ${CMAKE_CURRENT_BINARY_DIR}/Settings2D.cfg
  BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/SettingsAcquisition.cfg
             ${CMAKE_CURRENT_BINARY_DIR}/Settings.cfg
             ${CMAKE_CURRENT_BINARY_DIR}/Settings2DAcquisition.cfg
             ${CMAKE_CURRENT_BINARY_DIR}/Settings2D.cfg
             ${CMAKE_CURRENT_BINARY_DIR}/generated_headers/SettingsAcquisitionConfigUtils.h
             ${CMAKE_CURRENT_BINARY_DIR}/generated_headers/SettingsConfigUtils.h
             ${CMAKE_CURRENT_BINARY_DIR}/generated_headers/Settings2DAcquisitionConfigUtils.h
             ${CMAKE_CURRENT_BINARY_DIR}/generated_headers/Settings2DConfigUtils.h
  COMMENT "Running generator"
)

set(LIBRARY_NAME ${PROJECT_NAME})

generate_dynamic_reconfigure_options(
  ${CMAKE_CURRENT_BINARY_DIR}/Settings.cfg
  ${CMAKE_CURRENT_BINARY_DIR}/SettingsAcquisition.cfg
  ${CMAKE_CURRENT_BINARY_DIR}/Settings2D.cfg
  ${CMAKE_CURRENT_BINARY_DIR}/Settings2DAcquisition.cfg
)
add_dependencies(${PROJECT_NAME}_gencfg ${GENERATOR_TARGET_NAME})
add_service_files(
  DIRECTORY
  srv
  FILES
  Capture.srv
  CaptureAndSave.srv
  Capture2D.srv
  CaptureAssistantSuggestSettings.srv
  LoadSettingsFromFile.srv
  LoadSettings2DFromFile.srv
  CameraInfoModelName.srv
  CameraInfoSerialNumber.srv
  IsConnected.srv
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
)
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs nodelet
)

# The catkin functions above sets directory-level include directories for the current
# directory. But the include directories are not set as SYSTEM. Unset the include
# directories that are set by catkin above, and instead manually add these as SYSTEM
# include directories to each target as needed
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES "")

# Library
add_library(
  ${LIBRARY_NAME}
  src/zivid_camera.cpp
  src/capture_settings_controller.cpp
)
turn_on_compiler_warnings_if_enabled(${LIBRARY_NAME})
target_include_directories(
  ${LIBRARY_NAME}
  PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}/generated_headers/
)
target_include_directories(
  ${LIBRARY_NAME}
  SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
target_compile_definitions(
  ${LIBRARY_NAME}
  PRIVATE
  "ZIVID_ROS_DRIVER_VERSION=\"${${PROJECT_NAME}_VERSION}\""
)
target_link_libraries(${LIBRARY_NAME} PUBLIC ${catkin_LIBRARIES})
target_link_libraries(${LIBRARY_NAME} PRIVATE Zivid::Core)
add_dependencies(
  ${LIBRARY_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

# Node
set(NODE_NAME ${PROJECT_NAME}_node)
add_executable(${NODE_NAME} src/node.cpp)
turn_on_compiler_warnings_if_enabled(${NODE_NAME})
target_include_directories(${NODE_NAME} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(${NODE_NAME} ${LIBRARY_NAME} ${catkin_LIBRARIES})
add_dependencies(
  ${NODE_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${LIBRARY_NAME}
)

# Nodelet
set(NODELET_NAME ${PROJECT_NAME}_nodelet)
add_library(${NODELET_NAME} src/nodelet.cpp)
turn_on_compiler_warnings_if_enabled(${NODELET_NAME})
target_include_directories(
  ${NODELET_NAME}
  SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
target_include_directories(
  ${NODELET_NAME}
  PRIVATE
  include
)
target_link_libraries(${NODELET_NAME} ${LIBRARY_NAME} ${catkin_LIBRARIES})
add_dependencies(
  ${NODELET_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${LIBRARY_NAME}
)

#############
## Install ##
#############

install(
  TARGETS ${LIBRARY_NAME} ${NODE_NAME} ${NODELET_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  set(TEST_TARGET_NAME ${PROJECT_NAME}_test)
  add_executable(${TEST_TARGET_NAME} EXCLUDE_FROM_ALL test/test_zivid_camera.cpp)
  turn_on_compiler_warnings_if_enabled(${TEST_TARGET_NAME})
  target_include_directories(
    ${TEST_TARGET_NAME}
    SYSTEM PRIVATE
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${TEST_TARGET_NAME} ${LIBRARY_NAME} ${GTEST_LIBRARIES} Zivid::Core ${catkin_LIBRARIES})
  add_rostest(test/test_zivid_camera.test DEPENDENCIES ${TEST_TARGET_NAME})

endif()
