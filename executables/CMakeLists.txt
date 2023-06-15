link_libraries(${OpenCV_LIBS})


# add_executable(test_dataset_reader test_dataset_reader.cpp
#     ../src/dataset.cpp
#     ../src/drawer.cpp
# )

# target_link_libraries(test_dataset_reader opencv_highgui)


# add_executable(test_map_correspondence test_map_correspondence.cpp
#     ../src/dataset.cpp
#     ../src/drawer.cpp
#     ../src/utils.cpp
# )

# target_link_libraries(test_map_correspondence opencv_highgui)


# add_executable(test_normals_estimation test_normals_estimation.cpp
#     ../src/dataset.cpp
#     ../src/drawer.cpp
#     ../src/utils.cpp
# )

# target_link_libraries(test_normals_estimation opencv_highgui)

# add_executable(test_icp_solver test_icp_solver.cpp
#     ../src/dataset.cpp
#     ../src/drawer.cpp
#     ../src/utils.cpp
#     ../src/icp_solver.cpp
# )

# target_link_libraries(test_icp_solver opencv_highgui)


add_executable(test_multi_icp_solver test_multi_icp_solver.cpp
    ../src/dataset.cpp
    ../src/drawer.cpp
    ../src/utils.cpp
    ../src/multi_icp_solver.cpp
)

target_link_libraries(test_multi_icp_solver opencv_highgui)