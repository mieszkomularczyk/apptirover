cmake_minimum_required(VERSION 3.5)
project(gst_to_image)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Suppress all warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)
include_directories(
  include
  ${GST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Shared library (component) for IPC composition
add_library(gst_to_image_component SHARED src/gst_to_image.cpp)
ament_target_dependencies(gst_to_image_component
  rclcpp rclcpp_components sensor_msgs)
target_link_libraries(gst_to_image_component
  ${GST_LIBRARIES}
  ${OpenCV_LIBS}
)
rclcpp_components_register_nodes(gst_to_image_component "GstToImage")

# Standalone executable (with IPC enabled by default)
add_executable(gst_to_image src/gst_to_image.cpp)
ament_target_dependencies(gst_to_image
  rclcpp rclcpp_components sensor_msgs)
target_link_libraries(gst_to_image
  ${GST_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS gst_to_image_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gst_to_image
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
