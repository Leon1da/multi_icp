
### Dependencies (Ubuntu 20.04 LTS)
The following packages are required <br>
Before installing anything one should make sure what packages are already installed on the system!

CMake build utilities:

    sudo apt install build-essential cmake
    
OpenCV 2/3: https://opencv.org

    sudo apt install libopencv-dev
    
Eigen3: http://eigen.tuxfamily.org

    sudo apt install libeigen3-dev

### Compilation
From the system console, execute the build sequence (out of source build):

    mkdir build
    cd build
    cmake ..
    make
    
### Execution
The project provides the following binaries in the `build/executables` folder:
- `./test_dataset_reader`: show the dataset
- `./test_map_correspondence`: testing of the kdtree functionalities.
