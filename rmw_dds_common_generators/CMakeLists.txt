cmake_minimum_required(VERSION 3.5)

project(rmw_dds_common_generators NONE)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(ament_cmake_core)
ament_export_dependencies(rosidl_cmake)
ament_export_dependencies(rosidl_generator_c)
ament_export_dependencies(rosidl_generator_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rmw_dds_common_generators-extras.cmake.in"
)
