cmake_minimum_required(VERSION 3.26.3)

#ds project name
project(multi_icp)

#ds set cmake build flags
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-Wall -std=c++17 -O3 -DNDEBUG") #ds enable all warnings, c++11 features, enable maximum optimization and disable debug symbols
# set(CMAKE_CXX_FLAGS "-Wall -std=c++17 -DNDEBUG") #ds enable all warnings, c++11 features, enable maximum optimization and disable debug symbols


#ds find and set up dependencies
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})



#ds set internal includes (our files)
include_directories(src)

#ds add build targets
add_subdirectory(executables)

