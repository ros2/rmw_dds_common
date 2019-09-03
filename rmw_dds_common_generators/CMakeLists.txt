cmake_minimum_required(VERSION 3.5)

project(rmw_dds_common_generators NONE)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rmw_dds_common_generators-extras.cmake.in"
)
