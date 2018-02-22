cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ppf_recoginition)


set(PCL_DIR "D:/Program Files/PCL 1.8.0/cmake")
#set(PCL_DIR "D:/Program Files/PCL/cmake")


#OpenCV library
#set(OpenCV_DIR "D:/VC_Lib/opencv/build")
#find_package(OpenCV 3.0 REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

# posifix
set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd")

#OpenCV contrib modules, these are built individually
#add_subdirectory(D:/Documents/GitHub/opencv_contrib/modules/rgbd/   rgbd)

#PCL library directories
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# source list and source includes
include_directories(.)
aux_source_directory(. zyk_ppf_src)

#add target libs
add_library(zyk_ppf SHARED ${zyk_ppf_src})

#python bindings
aux_source_directory(./python/  zyk_ppf_python_src)
add_library(zyk_ppf_python SHARED ${zyk_ppf_python_src})

target_link_libraries(zyk_ppf ${PCL_LIBRARIES})
target_link_libraries(zyk_ppf_python zyk_ppf ${PCL_LIBRARIES})

#apps and tools
add_subdirectory(app)
add_subdirectory(tests)