cmake_minimum_required(VERSION 3.18)
project(yolo LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES "72;87")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(dog_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

set(YOLOS_INCLUDE_DIR "/workspace/libs/YOLOs-CPP-TensorRT/include")

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS
    /usr/include/aarch64-linux-gnu
    /usr/include/x86_64-linux-gnu
    /usr/include
)

find_library(NVINFER_LIB nvinfer
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
)

if(NOT NVINFER_LIB)
  message(FATAL_ERROR "TensorRT nvinfer library not found")
endif()

add_library(yolo_cuda_preprocessing STATIC
  "${YOLOS_INCLUDE_DIR}/yolos/core/cuda_preprocessing.cu"
)
target_include_directories(yolo_cuda_preprocessing PRIVATE
  "${YOLOS_INCLUDE_DIR}"
  "${TENSORRT_INCLUDE_DIR}"
)
target_link_libraries(yolo_cuda_preprocessing PRIVATE CUDA::cudart)
set_target_properties(yolo_cuda_preprocessing PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

set(YOLO_DEPS
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  vision_msgs
  foxglove_msgs
  dog_msgs
)

add_library(yolo_component SHARED src/yolo_node.cpp)
target_include_directories(yolo_component PRIVATE
  "${YOLOS_INCLUDE_DIR}"
  "${TENSORRT_INCLUDE_DIR}"
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(yolo_component ${YOLO_DEPS})
target_link_libraries(yolo_component
  ${NVINFER_LIB}
  CUDA::cudart
  yolo_cuda_preprocessing
  ${OpenCV_LIBS}
)
rclcpp_components_register_nodes(yolo_component "YoloNode")

add_executable(yolo_node src/yolo_node.cpp)
target_include_directories(yolo_node PRIVATE
  "${YOLOS_INCLUDE_DIR}"
  "${TENSORRT_INCLUDE_DIR}"
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(yolo_node ${YOLO_DEPS})
target_link_libraries(yolo_node
  ${NVINFER_LIB}
  CUDA::cudart
  yolo_cuda_preprocessing
  ${OpenCV_LIBS}
)

install(TARGETS yolo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS yolo_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
