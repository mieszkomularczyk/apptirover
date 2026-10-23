cmake_minimum_required(VERSION 3.8)
project(dog_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

# Collect all .msg files in msg/ directory
file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")

# Collect all .srv files in srv/ directory
file(GLOB srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
