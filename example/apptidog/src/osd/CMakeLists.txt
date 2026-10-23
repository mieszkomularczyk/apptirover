cmake_minimum_required(VERSION 3.8)
project(osd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPS
  rclcpp
  rclcpp_components
  sensor_msgs
  foxglove_msgs
  cv_bridge
)

# --- Shared library (component) for IPC composition ---
add_library(osd_component SHARED src/osd_node.cpp)
target_include_directories(osd_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(osd_component ${DEPS})
target_link_libraries(osd_component ${OpenCV_LIBS})
rclcpp_components_register_nodes(osd_component "osd::OsdNode")

# --- Standalone executable ---
add_executable(osd_node src/main.cpp src/osd_node.cpp)
target_include_directories(osd_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(osd_node ${DEPS})
target_link_libraries(osd_node ${OpenCV_LIBS})

# --- Install ---
install(TARGETS osd_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS osd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
