cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ppf_recoginition)

add_compile_options(-std=c++11)
#set(PCL_DIR "D:/Program Files/PCL 1.8.0/cmake")
#set(PCL_DIR "D:/Program Files/PCL/cmake")
set(PCL_DIR /usr/local/share/pcl-1.8)

#OpenCV library
#set(OpenCV_DIR "D:/VC_Lib/opencv/build")
#set(OpenCV_DIR "/opt/")
#find_package(OpenCV 3.0 REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

# posifix
set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd")

#OpenCV contrib modules, these are built individually
#add_subdirectory(D:/Documents/GitHub/opencv_contrib/modules/rgbd/   rgbd)

#PCL library directories
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add common utils
aux_source_directory(common_utils common_utils_src)
add_library(common_utils SHARED ${common_utils_src})
include_directories(common_utils)

# add pcl utils
add_subdirectory(pcl_utils)
# aux_source_directory(pcl_utils pcl_utils_src)
# add_library(pcl_utils SHARED ${pcl_utils_src})
# target_link_libraries(pcl_utils common_utils ${PCL_LIBRARIES})
include_directories(pcl_utils)

# add zyk ppf lib
aux_source_directory(zyk_ppf zyk_ppf_src)
add_library(zyk_ppf SHARED ${zyk_ppf_src})
target_link_libraries(zyk_ppf ${PCL_LIBRARIES} pcl_utils common_utils)
include_directories(zyk_ppf)

# #python bindings
# aux_source_directory(./python/  zyk_ppf_python_src)
# add_library(zyk_ppf_python SHARED ${zyk_ppf_python_src})
INCLUDE_DIRECTORIES(.)
#cv utils
add_subdirectory(cv_utils)

#apps and tools
add_subdirectory(app)
add_subdirectory(tests)
