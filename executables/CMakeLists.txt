link_libraries(${OpenCV_LIBS})


add_executable(test_multi_icp_complete test_multi_icp_complete.cpp
    
)

target_link_libraries(test_multi_icp_complete opencv_highgui cholmod)
