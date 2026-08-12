cmake_minimum_required(VERSION 3.10)
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
  cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "LINK_TARGETS")
  set(target_name "${sample_name}_cpp")

  add_executable(${target_name} "src/${sample_name}.cpp")
  target_link_libraries(${target_name} PUBLIC ${ARG_LINK_TARGETS})
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

zivid_add_cpp_sample(sample_capture
  LINK_TARGETS
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${std_srvs_TARGETS}
)
zivid_add_cpp_sample(sample_capture_2d
  LINK_TARGETS
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${std_srvs_TARGETS}
)
zivid_add_cpp_sample(sample_capture_and_save
  LINK_TARGETS
    rclcpp::rclcpp
    ${zivid_interfaces_TARGETS}
)
zivid_add_cpp_sample(sample_capture_assistant
  LINK_TARGETS
    rclcpp::rclcpp
    ${zivid_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${std_srvs_TARGETS}
)
zivid_add_cpp_sample(sample_capture_with_settings_from_file
  LINK_TARGETS
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${std_srvs_TARGETS}
)
zivid_add_cpp_sample(sample_capture_and_detect_calibration_board
  LINK_TARGETS
    rclcpp::rclcpp
    ${zivid_interfaces_TARGETS}
)
zivid_add_cpp_sample(sample_capture_and_detect_markers
  LINK_TARGETS
    rclcpp::rclcpp
    ${zivid_interfaces_TARGETS}
)
zivid_add_cpp_sample(sample_hand_eye_calibration
  LINK_TARGETS
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
    ${zivid_interfaces_TARGETS}
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
    tf2_ros::static_transform_broadcaster_node
)
zivid_add_cpp_sample(sample_infield_correction
  LINK_TARGETS
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
    ${zivid_interfaces_TARGETS}
)
zivid_add_cpp_sample(sample_intrinsics
  LINK_TARGETS
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${std_srvs_TARGETS}
)
zivid_add_cpp_sample(sample_projection
  LINK_TARGETS
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
    ${zivid_interfaces_TARGETS}
)
zivid_add_cpp_sample(sample_project_and_capture
  LINK_TARGETS
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${zivid_interfaces_TARGETS}
)

add_executable(capture_2d_trigger src/capture_2d_trigger.cpp)
ament_target_dependencies(capture_2d_trigger rclcpp std_srvs)
install(TARGETS capture_2d_trigger DESTINATION lib/${PROJECT_NAME})

zivid_add_cpp_sample(sample_with_sdk_capture_and_load_frame
  LINK_TARGETS
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${std_srvs_TARGETS}
    ${zivid_interfaces_TARGETS}
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
    tf2_ros::static_transform_broadcaster_node
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
