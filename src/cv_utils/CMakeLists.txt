cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
add_definitions(-std=c++11)
project(cv_utils)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(.)
message("opencv found in ${OpenCV_INCLUDE_DIRS}")
add_definitions(-DCVAPI_EXPORTS)
aux_source_directory(.  src)
add_library(cv_utils  SHARED ${src})
target_link_libraries(cv_utils ${OpenCV_LIBS})

set(CV_UTILS)
