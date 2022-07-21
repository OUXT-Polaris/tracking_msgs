cmake_minimum_required(VERSION 3.8)
project(perception_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(unique_identifier_msgs)
find_package(vision_msgs)

find_package(rosidl_default_generators REQUIRED)

file(GLOB SOURCE_MSGS_SRVS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg srv/*.srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SOURCE_MSGS_SRVS}
  DEPENDENCIES
    unique_identifier_msgs
    vision_msgs
  ADD_LINTER_TESTS
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(rosidl_default_runtime)

ament_package()
