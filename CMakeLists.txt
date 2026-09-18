cmake_minimum_required(VERSION 3.10)
project(pgr_fastlio_gnss_localization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# multithreaded ikd-Tree update (same policy as FAST-LIO upstream)
include(ProcessorCount)
ProcessorCount(N)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    if(N GREATER 4)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=3)
    elseif(N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
    else()
        add_definitions(-DMP_PROC_NUM=1)
    endif()
else()
    add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PythonLibs REQUIRED)

# Livox SDK2 (sdk_core only; samples are skipped)
add_subdirectory(third_party/Livox-SDK2/sdk_core livox_sdk2_build EXCLUDE_FROM_ALL)

set(FASTLIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/fastlio)

add_executable(lio_node
    src/app_main.cpp
    src/mid360/lidar_source.cpp
    ${FASTLIO_DIR}/src/preprocess.cpp
    ${FASTLIO_DIR}/include/ikd-Tree/ikd_Tree.cpp
)

target_include_directories(lio_node PRIVATE
    ${FASTLIO_DIR}/include
    ${FASTLIO_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mid360
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pose
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logger
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

target_link_libraries(lio_node
    livox_lidar_sdk_static
    ${PCL_LIBRARIES}
    ${PYTHON_LIBRARIES}
    pthread
)
