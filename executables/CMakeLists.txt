link_libraries(${OpenCV_LIBS})


add_executable(test_multi_icp_complete test_multi_icp_complete.cpp
    
)

target_link_libraries(test_multi_icp_complete opencv_highgui cholmod)



add_executable(test_multi_icp_complete_v2 test_multi_icp_complete_v2.cpp
    
)

target_link_libraries(test_multi_icp_complete_v2 opencv_highgui cholmod)
