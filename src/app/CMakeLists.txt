cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

#add_executable(ply2pcd ply2pcd.cpp)
#target_link_libraries(ply2pcd ${PCL_LIBRARIES})

#add_executable(txt2pcd txt2pcd.cpp)
#target_link_libraries(txt2pcd ${PCL_LIBRARIES})

add_executable(ppf_match ppf_match.cpp)
target_link_libraries(ppf_match zyk_ppf ${PCL_LIBRARIES} pcl_utils)

add_executable(ppf_train ppf_train.cpp)
target_link_libraries(ppf_train zyk_ppf ${PCL_LIBRARIES} pcl_utils)

add_executable(ppf_train2 ppf_train_ver2.cpp)
target_link_libraries(ppf_train2 zyk_ppf ${PCL_LIBRARIES} pcl_utils)

add_executable(mesh_sampling mesh_sampling.cpp)
target_link_libraries(mesh_sampling ${PCL_LIBRARIES} pcl_utils)
