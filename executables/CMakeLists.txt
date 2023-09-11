link_libraries(${OpenCV_LIBS})

add_executable(test_multi_icp_my_kdtree test_multi_icp_my_kdtree.cpp
    ../src/dataset.cpp
    ../src/drawer.cpp
    ../src/utils.cpp
    ../src/multi_icp_solver.cpp
)

target_link_libraries(test_multi_icp_my_kdtree opencv_highgui cholmod)

add_executable(test_multi_icp_rp_kdtree test_multi_icp_rp_kdtree.cpp
    ../src/dataset.cpp
    ../src/drawer.cpp
    ../src/utils.cpp
    ../src/multi_icp_solver.cpp
    
)

target_link_libraries(test_multi_icp_rp_kdtree opencv_highgui cholmod)


# add_executable(test_dataset_reader test_dataset_reader.cpp
#     ../src/dataset.cpp
#     ../src/drawer.cpp
# )

# target_link_libraries(test_dataset_reader opencv_highgui)


# add_executable(test_points_correspondences test_points_correspondences.cpp
#     ../src/dataset.cpp
#     ../src/drawer.cpp
#     ../src/utils.cpp
# )

# target_link_libraries(test_points_correspondences opencv_highgui)

# add_executable(test_poses_correspondences test_poses_correspondences.cpp
#     ../src/dataset.cpp
#     ../src/drawer.cpp
#     ../src/utils.cpp
# )

# target_link_libraries(test_poses_correspondences opencv_highgui)


# add_executable(test_normals test_normals.cpp
#     ../src/dataset.cpp
#     ../src/drawer.cpp
#     ../src/utils.cpp
# )

# target_link_libraries(test_normals opencv_highgui)

