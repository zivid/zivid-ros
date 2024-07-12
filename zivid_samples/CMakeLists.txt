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

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(zivid_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(sample_capture_cpp src/sample_capture.cpp)
ament_target_dependencies(sample_capture_cpp rclcpp zivid_interfaces sensor_msgs std_srvs ament_index_cpp)
install(TARGETS sample_capture_cpp DESTINATION lib/${PROJECT_NAME})

add_executable(sample_capture_2d_cpp src/sample_capture_2d.cpp)
ament_target_dependencies(sample_capture_2d_cpp rclcpp zivid_interfaces sensor_msgs std_srvs ament_index_cpp)
install(TARGETS sample_capture_2d_cpp DESTINATION lib/${PROJECT_NAME})

add_executable(sample_capture_assistant_cpp src/sample_capture_assistant.cpp)
ament_target_dependencies(sample_capture_assistant_cpp rclcpp zivid_interfaces sensor_msgs std_srvs ament_index_cpp)
install(TARGETS sample_capture_assistant_cpp DESTINATION lib/${PROJECT_NAME})

add_executable(sample_capture_and_save_cpp src/sample_capture_and_save.cpp)
ament_target_dependencies(sample_capture_and_save_cpp rclcpp zivid_interfaces)
install(TARGETS sample_capture_and_save_cpp DESTINATION lib/${PROJECT_NAME})

add_executable(sample_capture_with_settings_from_file_cpp src/sample_capture_with_settings_from_file.cpp)
ament_target_dependencies(sample_capture_with_settings_from_file_cpp rclcpp zivid_interfaces sensor_msgs std_srvs ament_index_cpp)
install(TARGETS sample_capture_with_settings_from_file_cpp DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/sample_capture.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/sample_capture_2d.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/sample_capture_and_save.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/sample_capture_assistant.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/sample_capture_with_settings_from_file.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # TODO enable linting of the samples
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY settings DESTINATION share/${PROJECT_NAME})

ament_package()
