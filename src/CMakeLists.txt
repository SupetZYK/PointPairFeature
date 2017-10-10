cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ppf_recoginition)


set(PCL_DIR "D:/Program Files/PCL/cmake/")
#PCL library directories
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(.)

set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd")
# source list
aux_source_directory(. zyk_ppf_src)
add_library(zyk_ppf SHARED ${zyk_ppf_src})

#python library
aux_source_directory(./python/  zyk_ppf_python_src)
add_library(zyk_ppf_python SHARED ${zyk_ppf_python_src})

target_link_libraries(zyk_ppf ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(zyk_ppf_python zyk_ppf ${PCL_LIBRARIES})

add_subdirectory(app)