cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

add_executable(test_detectors  test_detectors.cpp)
target_link_libraries(test_detectors zyk_ppf)

add_executable(test_detector_train  test_detector_train.cpp)
target_link_libraries(test_detector_train zyk_ppf)
