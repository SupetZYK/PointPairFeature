cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

aux_source_directory(. test_list)

foreach(test_file_namme ${test_list})
get_filename_component(test_name ${test_file_namme} NAME_WE)
add_executable(${test_name}   ${test_file_namme})
message("current test is: ${test_name}")
message("current test file is: ${test_file_namme}")
target_link_libraries(${test_name} zyk_ppf rgbd ${PCL_LIBRARIES} ${OpenCV_LIBS})
endforeach(test_file_namme ${test_list})