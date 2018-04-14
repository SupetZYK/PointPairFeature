cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

# aux_source_directory(. test_list)

# foreach(test_file_namme ${test_list})
# get_filename_component(test_name ${test_file_namme} NAME_WE)
# add_executable(${test_name}   ${test_file_namme})
# message("current test is: ${test_name}")
# message("current test file is: ${test_file_namme}")
# target_link_libraries(${test_name} zyk_ppf rgbd ${PCL_LIBRARIES} ${OpenCV_LIBS})
# endforeach(test_file_namme ${test_list})

add_executable(test_detectors  test_Detectors.cpp)
target_link_libraries(test_detectors zyk_ppf)

add_executable(test_detector_train  test_Detector_train.cpp)
target_link_libraries(test_detector_train zyk_ppf)

add_executable(test_neiboriterator test_neiborIterator.cpp)
target_link_libraries(test_neiboriterator zyk_ppf)

add_executable(test_normal  test_normal_compute.cpp)
target_link_libraries(test_normal ${PCL_LIBRARIES} zyk_ppf)


#add_executable(test_read_depth test_read_depth.cpp)
#target_link_libraries(test_read_depth ${PCL_LIBRARIES} rgbd zyk_ppf)

