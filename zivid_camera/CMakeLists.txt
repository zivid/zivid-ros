cmake_minimum_required(VERSION 3.5)
project(zivid_camera LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(zivid_interfaces REQUIRED)

find_package(Zivid 2.9.0 COMPONENTS Core REQUIRED)
message(STATUS "Found Zivid SDK version ${Zivid_VERSION}")

set(ZIVID_DRIVER_COMPONENT_NAME ${PROJECT_NAME}_component)

add_library(
  ${ZIVID_DRIVER_COMPONENT_NAME}
  SHARED
  src/zivid_camera.cpp
)

turn_on_compiler_warnings_if_enabled(${ZIVID_DRIVER_COMPONENT_NAME})
target_include_directories(${ZIVID_DRIVER_COMPONENT_NAME} PUBLIC include)

rclcpp_components_register_node(${ZIVID_DRIVER_COMPONENT_NAME}
                                PLUGIN
                                "zivid_camera::ZividCamera"
                                EXECUTABLE
                                "zivid_camera")

target_link_libraries(${ZIVID_DRIVER_COMPONENT_NAME} Zivid::Core)
ament_target_dependencies(${ZIVID_DRIVER_COMPONENT_NAME}
                          rclcpp_components
                          std_msgs
                          std_srvs
                          sensor_msgs
                          image_transport
                          zivid_interfaces)

target_compile_definitions(${ZIVID_DRIVER_COMPONENT_NAME} PRIVATE "ZIVID_CAMERA_ROS_EXPORTING")

#############
## Install ##
#############

install(TARGETS ${ZIVID_DRIVER_COMPONENT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  set(TEST_TARGET_NAME ${PROJECT_NAME}_test)

  ament_add_gtest(${TEST_TARGET_NAME} test/test_zivid_camera.cpp TIMEOUT 480)
  ament_target_dependencies(${TEST_TARGET_NAME}
      rclcpp
      std_msgs
      sensor_msgs
      image_transport
      zivid_interfaces
  )
  target_link_libraries(${TEST_TARGET_NAME} Zivid::Core ${ZIVID_DRIVER_COMPONENT_NAME})

endif()

ament_package()