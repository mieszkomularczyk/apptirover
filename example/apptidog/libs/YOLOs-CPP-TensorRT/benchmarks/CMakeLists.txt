# ============================================================================
# YOLO Unified Benchmark - TensorRT CMake Configuration
# ============================================================================
cmake_minimum_required(VERSION 3.18)
project(yolo_unified_benchmark
    VERSION 3.0.0
    DESCRIPTION "Comprehensive benchmarking for YOLO TensorRT models"
    LANGUAGES CXX CUDA
)

# ============================================================================
# C++ Standard and Compiler Settings
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# ============================================================================
# Build Type
# ============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================================
# Compiler Warnings and Optimization
# ============================================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-missing-field-initializers)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3 -DNDEBUG)
        option(ENABLE_NATIVE_ARCH "Enable native architecture optimizations" ON)
        if(ENABLE_NATIVE_ARCH)
            add_compile_options(-march=native)
            message(STATUS "Native architecture optimizations enabled")
        endif()
        option(ENABLE_LTO "Enable Link Time Optimization" OFF)
        if(ENABLE_LTO)
            set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
            message(STATUS "LTO enabled")
        endif()
    endif()
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-g -O0)
        add_compile_definitions(DEBUG)
    endif()
elseif(MSVC)
    add_compile_options(/W4 /permissive-)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(/O2 /GL)
    endif()
endif()

# ============================================================================
# CUDA
# ============================================================================
find_package(CUDAToolkit REQUIRED)
message(STATUS "CUDA Toolkit: ${CUDAToolkit_VERSION}")

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES "75;80;86;89;90")
endif()

# ============================================================================
# TensorRT
# ============================================================================
if(NOT DEFINED TENSORRT_DIR)
    set(TENSORRT_SEARCH_PATHS
        "/usr/lib/x86_64-linux-gnu"
        "/usr/lib/aarch64-linux-gnu"
        "/usr/local/tensorrt"
        "$ENV{TENSORRT_DIR}"
    )
    foreach(path ${TENSORRT_SEARCH_PATHS})
        if(EXISTS "${path}/libnvinfer.so" OR EXISTS "${path}/lib/libnvinfer.so")
            set(TENSORRT_DIR "${path}")
            break()
        endif()
    endforeach()
endif()

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS ${TENSORRT_DIR}/include /usr/include/x86_64-linux-gnu /usr/include/aarch64-linux-gnu /usr/include
)
find_library(NVINFER_LIB nvinfer
    HINTS ${TENSORRT_DIR}/lib ${TENSORRT_DIR} /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
)

if(NOT NVINFER_LIB)
    message(FATAL_ERROR
        "TensorRT not found. Set -DTENSORRT_DIR=/path/to/tensorrt"
    )
endif()

message(STATUS "TensorRT: ${NVINFER_LIB}")

# ============================================================================
# OpenCV
# ============================================================================
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs videoio)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ============================================================================
# CUDA Preprocessing Library
# ============================================================================
add_library(cuda_preprocessing_bench STATIC
    "${CMAKE_SOURCE_DIR}/../include/yolos/core/cuda_preprocessing.cu"
)
target_include_directories(cuda_preprocessing_bench PRIVATE
    "${CMAKE_SOURCE_DIR}/../include"
    "${TENSORRT_INCLUDE_DIR}"
)
target_link_libraries(cuda_preprocessing_bench PRIVATE CUDA::cudart)
set_target_properties(cuda_preprocessing_bench PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
)

# ============================================================================
# Benchmark Executable
# ============================================================================
add_executable(yolo_unified_benchmark yolo_unified_benchmark.cpp)

target_include_directories(yolo_unified_benchmark PRIVATE
    "${CMAKE_SOURCE_DIR}/../include"
    "${TENSORRT_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(yolo_unified_benchmark PRIVATE
    ${OpenCV_LIBS}
    ${NVINFER_LIB}
    CUDA::cudart
    cuda_preprocessing_bench
    pthread
    dl
)

if(UNIX)
    set_target_properties(yolo_unified_benchmark PROPERTIES
        BUILD_RPATH "$ORIGIN"
        INSTALL_RPATH "$ORIGIN"
    )
endif()

# ============================================================================
# Installation
# ============================================================================
install(TARGETS yolo_unified_benchmark RUNTIME DESTINATION bin)

# ============================================================================
# Summary
# ============================================================================
message(STATUS "")
message(STATUS "=== YOLO Unified Benchmark Configuration ===")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  Build Type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:   ${CMAKE_CXX_STANDARD}")
message(STATUS "  OpenCV:         ${OpenCV_VERSION}")
message(STATUS "  TensorRT:       ${NVINFER_LIB}")
message(STATUS "  CUDA Toolkit:   ${CUDAToolkit_VERSION}")
message(STATUS "")
