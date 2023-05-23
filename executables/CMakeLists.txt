link_libraries(${OpenCV_LIBS})


add_executable(test_dataset_reader test_dataset_reader.cpp
    ../src/dataset.cpp
    ../src/map.cpp
    ../src/drawer.cpp
)

target_link_libraries(test_dataset_reader opencv_highgui)


add_executable(test_map_correspondence test_map_correspondence.cpp
    ../src/dataset.cpp
    ../src/map.cpp
    ../src/drawer.cpp
    ../src/correspondence_finder.cpp
)

target_link_libraries(test_map_correspondence opencv_highgui)