cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

aux_source_directory(. app_list)


foreach(app_file_namme ${app_list})
get_filename_component(app_name ${app_file_namme} NAME_WE)
add_executable(${app_name}   ${app_file_namme})
message("current app is: ${app_name}")
message("current app file is: ${app_file_namme}")
target_link_libraries(${app_name} zyk_ppf ${PCL_LIBRARIES})
endforeach(app_file_namme ${app_list})
