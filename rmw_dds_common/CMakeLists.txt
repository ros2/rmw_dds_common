cmake_minimum_required(VERSION 3.5)

project(rmw_dds_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_dds_common_generators REQUIRED)
find_package(rosidl_cmake REQUIRED)

ament_export_dependencies(ament_cmake_core)
ament_export_dependencies(rcpputils)
ament_export_dependencies(rcutils)
ament_export_dependencies(rmw)
ament_export_dependencies(rosidl_cmake)

rosidl_generate_interfaces(
  ${PROJECT_NAME}_interfaces
  "msg/Gid.idl"
  "msg/NodeCustomInfo.idl"
  "msg/ParticipantCustomInfo.idl"
  SKIP_GROUP_MEMBERSHIP_CHECK
)

add_library(${PROJECT_NAME} SHARED
  src/gid_utils.cpp
  src/node_cache.cpp
  src/rmw_context_impl.cpp
  src/topic_cache.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rcpputils"
  "rcutils"
  "rmw"
  "rosidl_generator_cpp")
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RMW_DDS_COMMON_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(rcpputils)
ament_export_dependencies(rcutils)
ament_export_dependencies(rmw)
ament_export_dependencies(rosidl_generator_cpp)

install(
  DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_node_cache test/test_node_cache.cpp)
  if(TARGET test_node_cache)
    target_link_libraries(test_node_cache ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_topic_cache test/test_topic_cache.cpp)
  if(TARGET test_topic_cache)
    target_link_libraries(test_topic_cache ${PROJECT_NAME})
  endif()
endif()

ament_package()
