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
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "DEPENDENCIES")
  set(target_name "${sample_name}_cpp")

  add_executable(${target_name} "src/${sample_name}.cpp")
  ament_target_dependencies(${target_name} ${ARG_DEPENDENCIES})
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

zivid_add_cpp_sample(sample_capture DEPENDENCIES rclcpp sensor_msgs std_srvs)
zivid_add_cpp_sample(sample_capture_2d DEPENDENCIES rclcpp sensor_msgs std_srvs)
zivid_add_cpp_sample(sample_capture_and_save DEPENDENCIES rclcpp zivid_interfaces)
zivid_add_cpp_sample(sample_capture_assistant DEPENDENCIES rclcpp zivid_interfaces sensor_msgs std_srvs)
zivid_add_cpp_sample(sample_capture_with_settings_from_file DEPENDENCIES rclcpp sensor_msgs std_srvs ament_index_cpp)
zivid_add_cpp_sample(sample_infield_correction DEPENDENCIES rclcpp std_srvs zivid_interfaces)

zivid_add_python_sample(sample_capture)
zivid_add_python_sample(sample_capture_2d)
zivid_add_python_sample(sample_capture_and_save)
zivid_add_python_sample(sample_capture_assistant)
zivid_add_python_sample(sample_capture_with_settings_from_file)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY settings DESTINATION share/${PROJECT_NAME})

ament_package()
