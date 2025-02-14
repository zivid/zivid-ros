cmake_minimum_required(VERSION 3.8)
project(zivid_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(zivid_interfaces REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/zivid_rviz_plugin/infield_correction.hpp
)

add_library(zivid_rviz_plugin src/infield_correction.cpp ${MOC_FILES})
target_include_directories(zivid_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(zivid_rviz_plugin
  rclcpp
  pluginlib
  rviz_common
  std_srvs
  zivid_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS zivid_rviz_plugin
  EXPORT export_zivid_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES "icons/classes/Zivid Infield Correction.png"
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_zivid_rviz_plugin)
pluginlib_export_plugin_description_file(rviz_common plugins.xml)

ament_package()
