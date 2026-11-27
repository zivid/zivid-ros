cmake_minimum_required(VERSION 3.10)
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
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(zivid_interfaces REQUIRED)

find_package(Zivid 2.15.0 COMPONENTS Core REQUIRED)
message(STATUS "Found Zivid SDK version ${Zivid_VERSION}")

# Optional support for publishing in CUDA-backed buffers (rosidl::Buffer with the CUDA backend),
# available from ROS 2 Lyrical when the cuda_buffer package is installed. See parameter
# `use_cuda_buffers`.
find_package(cuda_buffer QUIET)
if(cuda_buffer_FOUND AND Zivid_VERSION VERSION_GREATER_EQUAL 2.18)
  find_package(CUDAToolkit REQUIRED)
  set(ZIVID_CAMERA_WITH_CUDA_BUFFER ON)
  message(STATUS "Building with CUDA buffer support (cuda_buffer ${cuda_buffer_VERSION})")
  # cuda_buffer releases up to and including 0.1.2 keep the CUDA memory pool in a header-only static.
  # When the driver is loaded as a component library (ros2 run, component containers), the backend
  # plugin then uses a different pool instance than the driver, and subscribers in other processes
  # silently receive host copies instead of CUDA buffers. Fixed upstream in
  # ros2/rosidl_buffer_backends#12 by an out-of-line cuda_buffer_backend::get_or_create_global_pool.
  # The package version was not bumped with the fix, so check for the function instead.
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES cuda_buffer::cuda_buffer CUDA::cudart CUDA::cuda_driver)
  check_cxx_source_compiles("
    #include <cuda_buffer/cuda_buffer_impl.hpp>
    int main() { return cuda_buffer_backend::get_or_create_global_pool() == nullptr; }"
    ZIVID_CAMERA_CUDA_BUFFER_HAS_SHARED_POOL)
  unset(CMAKE_REQUIRED_LIBRARIES)
  if(NOT ZIVID_CAMERA_CUDA_BUFFER_HAS_SHARED_POOL)
    set(ZIVID_CAMERA_CUDA_BUFFER_HAS_DSO_POOL_ISSUE ON)
    message(WARNING
      "The installed cuda_buffer does not include the fix for "
      "https://github.com/ros2/rosidl_buffer_backends/pull/12. With use_cuda_buffers, "
      "subscribers in other processes will receive host copies instead of CUDA buffers.")
  endif()
else()
  set(ZIVID_CAMERA_WITH_CUDA_BUFFER OFF)
  message(STATUS "Building without CUDA buffer support (requires the cuda_buffer package and Zivid SDK 2.18 or newer)")
endif()

set(ZIVID_DRIVER_COMPONENT_NAME ${PROJECT_NAME}_component)

add_library(
  ${ZIVID_DRIVER_COMPONENT_NAME}
  SHARED
  src/camera_health_controller.cpp
  src/controller_interface.cpp
  src/hand_eye_calibration_controller.cpp
  src/infield_correction_controller.cpp
  src/detector_controller.cpp
  src/projection_controller.cpp
  src/utility.cpp
  src/zivid_camera.cpp
)

turn_on_compiler_warnings_if_enabled(${ZIVID_DRIVER_COMPONENT_NAME})
target_include_directories(${ZIVID_DRIVER_COMPONENT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(${ZIVID_DRIVER_COMPONENT_NAME}
                                PLUGIN
                                "zivid_camera::ZividCamera"
                                EXECUTABLE
                                "zivid_camera")

target_compile_definitions(${ZIVID_DRIVER_COMPONENT_NAME} PRIVATE "ZIVID_CAMERA_ROS_EXPORTING")

if(ZIVID_CAMERA_WITH_CUDA_BUFFER)
  target_sources(${ZIVID_DRIVER_COMPONENT_NAME} PRIVATE src/cuda_buffer_writer.cpp)
  target_compile_definitions(${ZIVID_DRIVER_COMPONENT_NAME} PRIVATE "ZIVID_CAMERA_WITH_CUDA_BUFFER")
  if(ZIVID_CAMERA_CUDA_BUFFER_HAS_DSO_POOL_ISSUE)
    target_compile_definitions(${ZIVID_DRIVER_COMPONENT_NAME} PRIVATE
      "ZIVID_CAMERA_CUDA_BUFFER_HAS_DSO_POOL_ISSUE")
  endif()
  target_link_libraries(${ZIVID_DRIVER_COMPONENT_NAME} PRIVATE
    cuda_buffer::cuda_buffer
    CUDA::cudart
    CUDA::cuda_driver
  )
endif()

target_link_libraries(${ZIVID_DRIVER_COMPONENT_NAME} PUBLIC
  ${diagnostic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${zivid_interfaces_TARGETS}
  image_transport::image_transport
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  Zivid::Core
)

#############
## Install ##
#############

set(ZIVID_DRIVER_DEPENDENCIES
  diagnostic_msgs
  image_transport
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  zivid_interfaces
)

set(ZIVID_DRIVER_TARGETS_EXPORT_NAME ${PROJECT_NAME}Targets)

install(TARGETS ${ZIVID_DRIVER_COMPONENT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(${ZIVID_DRIVER_TARGETS_EXPORT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp_components ${ZIVID_DRIVER_DEPENDENCIES})

install(
  TARGETS ${ZIVID_DRIVER_COMPONENT_NAME}
  EXPORT ${ZIVID_DRIVER_TARGETS_EXPORT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  set(TEST_TARGET_NAME ${PROJECT_NAME}_test)

  ament_add_gtest(${TEST_TARGET_NAME}
      test/test_zivid_camera.cpp
      test/test_camera_health.cpp
      test/test_detector.cpp
      test/test_hand_eye_calibration.cpp
      test/test_infield_correction.cpp
      TIMEOUT 1000
  )
  target_link_libraries(${TEST_TARGET_NAME}
      ${diagnostic_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${zivid_interfaces_TARGETS}
      image_transport::image_transport
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library
      Zivid::Core
      ${ZIVID_DRIVER_COMPONENT_NAME}
  )
  target_include_directories(${TEST_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/include)

  if(ZIVID_CAMERA_WITH_CUDA_BUFFER)
    set(CUDA_BUFFER_TEST_TARGET_NAME ${PROJECT_NAME}_cuda_buffer_test)
    ament_add_gtest(${CUDA_BUFFER_TEST_TARGET_NAME}
        test/test_cuda_buffer.cpp
        TIMEOUT 300
    )
    target_link_libraries(${CUDA_BUFFER_TEST_TARGET_NAME}
        ${sensor_msgs_TARGETS}
        ${std_srvs_TARGETS}
        cuda_buffer::cuda_buffer
        CUDA::cudart
        CUDA::cuda_driver
        rclcpp::rclcpp
        Zivid::Core
        ${ZIVID_DRIVER_COMPONENT_NAME}
    )
  endif()

endif()

ament_package()