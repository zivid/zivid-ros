cmake_minimum_required(VERSION 3.5)
project(zivid_samples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

function(zivid_add_cpp_sample sample_name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "DEPENDENCIES;LINK_TARGETS")
  set(target_name "${sample_name}_cpp")

  add_executable(${target_name} "src/${sample_name}.cpp")
  ament_target_dependencies(${target_name} ${ARG_DEPENDENCIES})
  target_link_libraries(${target_name} ${ARG_LINK_TARGETS})
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
endfunction()

function(zivid_add_python_sample sample_name)
  install(PROGRAMS "scripts/${sample_name}.py" DESTINATION lib/${PROJECT_NAME})
endfunction()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(zivid_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Zivid 2.15.0 COMPONENTS Core REQUIRED)
message(STATUS "Found Zivid SDK version ${Zivid_VERSION}")

zivid_add_cpp_sample(sample_capture DEPENDENCIES rclcpp sensor_msgs std_srvs)
zivid_add_cpp_sample(sample_capture_2d DEPENDENCIES rclcpp sensor_msgs std_srvs)
zivid_add_cpp_sample(sample_capture_and_save DEPENDENCIES rclcpp zivid_interfaces)
zivid_add_cpp_sample(sample_capture_assistant DEPENDENCIES rclcpp zivid_interfaces sensor_msgs std_srvs)
zivid_add_cpp_sample(sample_capture_with_settings_from_file DEPENDENCIES rclcpp sensor_msgs std_srvs ament_index_cpp)
zivid_add_cpp_sample(sample_capture_and_detect_calibration_board DEPENDENCIES rclcpp zivid_interfaces)
zivid_add_cpp_sample(sample_capture_and_detect_markers DEPENDENCIES rclcpp zivid_interfaces)
zivid_add_cpp_sample(sample_hand_eye_calibration DEPENDENCIES rclcpp std_srvs zivid_interfaces tf2 tf2_ros tf2_geometry_msgs)
zivid_add_cpp_sample(sample_infield_correction DEPENDENCIES rclcpp std_srvs zivid_interfaces)
zivid_add_cpp_sample(sample_intrinsics DEPENDENCIES rclcpp sensor_msgs std_srvs)
zivid_add_cpp_sample(sample_projection DEPENDENCIES rclcpp std_srvs zivid_interfaces)
zivid_add_cpp_sample(sample_project_and_capture DEPENDENCIES rclcpp std_srvs sensor_msgs zivid_interfaces)

zivid_add_cpp_sample(sample_with_sdk_capture_and_load_frame
  DEPENDENCIES
    rclcpp sensor_msgs std_srvs zivid_interfaces tf2 tf2_ros tf2_geometry_msgs
  LINK_TARGETS
    Zivid::Core
)

zivid_add_python_sample(sample_capture)
zivid_add_python_sample(sample_capture_2d)
zivid_add_python_sample(sample_capture_and_save)
zivid_add_python_sample(sample_capture_assistant)
zivid_add_python_sample(sample_capture_with_settings_from_file)
zivid_add_python_sample(sample_hand_eye_calibration)
zivid_add_python_sample(sample_infield_correction)
zivid_add_python_sample(sample_projection)
zivid_add_python_sample(sample_project_and_capture)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY settings DESTINATION share/${PROJECT_NAME})

ament_package()
