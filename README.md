
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
- `./test_multi_icp_complete`: run the optimization

### Gnuplot usage

cd build/executable/results
gnuplot

set pointsize 0.1
plot "<>_points.dat" lt rgb "#57B8FF", "<>_points_opt_.dat" lt rgb "#C2261E", "<>_poses.dat" lt rgb "#313638", "<>_poses_opt.dat" lt rgb "#11D07D"


# Test
## Kernel 0.1
#### Test 01
plot 
"2023_09_30_19_35_32_conf_it_30_kt_0.1_dm_1000_points.dat" lt rgb "#57B8FF", "2023_09_30_19_35_32_conf_it_30_kt_0.1_dm_1000_points_opt.dat" lt rgb "#C2261E", "2023_09_30_19_35_32_conf_it_30_kt_0.1_dm_1000_poses.dat" lt rgb "#313638", "2023_09_30_19_35_32_conf_it_30_kt_0.1_dm_1000_poses_opt.dat" lt rgb "#11D07D"

#### Test 02
plot 
"2023_09_30_19_40_28_conf_it_30_kt_0.1_dm_100_points.dat" lt rgb "#57B8FF", "2023_09_30_19_40_28_conf_it_30_kt_0.1_dm_100_points_opt.dat" lt rgb "#C2261E", "2023_09_30_19_40_28_conf_it_30_kt_0.1_dm_100_poses.dat" lt rgb "#313638", "2023_09_30_19_40_28_conf_it_30_kt_0.1_dm_100_poses_opt.dat" lt rgb "#11D07D"

#### Test 03
 plot 
 "2023_09_30_19_50_34_conf_it_30_kt_0.1_dm_10_points.dat" lt rgb "#57B8FF", "2023_09_30_19_50_34_conf_it_30_kt_0.1_dm_10_points_opt.dat" lt rgb "#C2261E", "2023_09_30_19_50_34_conf_it_30_kt_0.1_dm_10_poses.dat" lt rgb "#313638", "2023_09_30_19_50_34_conf_it_30_kt_0.1_dm_10_poses_opt.dat" lt rgb "#11D07D"

#### Test 04
 plot 
 "2023_09_30_20_25_08_conf_it_30_kt_0.1_dm_1_points.dat" lt rgb "#57B8FF", "2023_09_30_20_25_08_conf_it_30_kt_0.1_dm_1_points_opt.dat" lt rgb "#C2261E", "2023_09_30_20_25_08_conf_it_30_kt_0.1_dm_1_poses.dat" lt rgb "#313638", "2023_09_30_20_25_08_conf_it_30_kt_0.1_dm_1_poses_opt.dat" lt rgb "#11D07D"

#### Test 05
plot "2023_09_30_20_41_10_conf_it_30_kt_0.1_dm_0.1_points.dat" lt rgb "#57B8FF", "2023_09_30_20_41_10_conf_it_30_kt_0.1_dm_0.1_points_opt.dat" lt rgb "#C2261E", "2023_09_30_20_41_10_conf_it_30_kt_0.1_dm_0.1_poses.dat" lt rgb "#313638", "2023_09_30_20_41_10_conf_it_30_kt_0.1_dm_0.1_poses_opt.dat" lt rgb "#11D07D"
#### bad aula magna, observe the bottom-right part of the map, the angles became right angles, very nice

#### Test 06

plot "2023_09_30_20_56_56_conf_it_30_kt_0.1_dm_0.5_points.dat" lt rgb "#57B8FF", "2023_09_30_20_56_56_conf_it_30_kt_0.1_dm_0.5_points_opt.dat" lt rgb "#C2261E", "2023_09_30_20_56_56_conf_it_30_kt_0.1_dm_0.5_poses.dat" lt rgb "#313638", "2023_09_30_20_56_56_conf_it_30_kt_0.1_dm_0.5_poses_opt.dat" lt rgb "#11D07D"

#### Test 07

plot "2023_10_02_15_48_44_conf_it_30_kt_0.1_dm_0.8_points.dat" lt rgb "#57B8FF", "2023_10_02_15_48_44_conf_it_30_kt_0.1_dm_0.8_points_opt.dat" lt rgb "#C2261E", "2023_10_02_15_48_44_conf_it_30_kt_0.1_dm_0.8_poses.dat" lt rgb "#313638", "2023_10_02_15_48_44_conf_it_30_kt_0.1_dm_0.8_poses_opt.dat" lt rgb "#11D07D"
#### bad aula magna, artifacts and duplicated walls


## Kernel 0.01
#### Test 11
plot "2023_09_30_21_02_57_conf_it_30_kt_0.01_dm_1000_points.dat" lt rgb "#57B8FF", "2023_09_30_21_02_57_conf_it_30_kt_0.01_dm_1000_points_opt.dat" lt rgb "#C2261E", "2023_09_30_21_02_57_conf_it_30_kt_0.01_dm_1000_poses.dat" lt rgb "#313638", "2023_09_30_21_02_57_conf_it_30_kt_0.01_dm_1000_poses_opt.dat" lt rgb "#11D07D"

#### Test 12
plot "2023_09_30_21_15_45_conf_it_30_kt_0.01_dm_100_points.dat" lt rgb "#57B8FF", "2023_09_30_21_15_45_conf_it_30_kt_0.01_dm_100_points_opt.dat" lt rgb "#C2261E", "2023_09_30_21_15_45_conf_it_30_kt_0.01_dm_100_poses.dat" lt rgb "#313638", "2023_09_30_21_15_45_conf_it_30_kt_0.01_dm_100_poses_opt.dat" lt rgb "#11D07D"

#### Test 13
plot "2023_09_30_21_25_47_conf_it_30_kt_0.01_dm_10_points.dat" lt rgb "#57B8FF", "2023_09_30_21_25_47_conf_it_30_kt_0.01_dm_10_points_opt.dat" lt rgb "#C2261E", "2023_09_30_21_25_47_conf_it_30_kt_0.01_dm_10_poses.dat" lt rgb "#313638", "2023_09_30_21_25_47_conf_it_30_kt_0.01_dm_10_poses_opt.dat" lt rgb "#11D07D"

#### Test 14
plot "2023_09_30_21_33_02_conf_it_30_kt_0.01_dm_1_points.dat" lt rgb "#57B8FF", "2023_09_30_21_33_02_conf_it_30_kt_0.01_dm_1_points_opt.dat" lt rgb "#C2261E", "2023_09_30_21_33_02_conf_it_30_kt_0.01_dm_1_poses.dat" lt rgb "#313638", "2023_09_30_21_33_02_conf_it_30_kt_0.01_dm_1_poses_opt.dat" lt rgb "#11D07D"
#### Observe Aula Magna, very good wall, nice

#### Test 15
plot "2023_09_30_21_41_57_conf_it_30_kt_0.01_dm_0.1_points.dat" lt rgb "#57B8FF", "2023_09_30_21_41_57_conf_it_30_kt_0.01_dm_0.1_points_opt.dat" lt rgb "#C2261E", "2023_09_30_21_41_57_conf_it_30_kt_0.01_dm_0.1_poses.dat" lt rgb "#313638", "2023_09_30_21_41_57_conf_it_30_kt_0.01_dm_0.1_poses_opt.dat" lt rgb "#11D07D"

#### ok
#### Test 16
plot "2023_09_30_22_51_42_conf_it_30_kt_0.01_dm_0.5_points.dat" lt rgb "#57B8FF", "2023_09_30_22_51_42_conf_it_30_kt_0.01_dm_0.5_points_opt.dat" lt rgb "#C2261E", "2023_09_30_22_51_42_conf_it_30_kt_0.01_dm_0.5_poses.dat" lt rgb "#313638", "2023_09_30_22_51_42_conf_it_30_kt_0.01_dm_0.5_poses_opt.dat" lt rgb "#11D07D"

#### Test 17
plot "2023_10_01_15_39_51_conf_it_30_kt_0.01_dm_0.8_points.dat" lt rgb "black", "2023_10_01_15_39_51_conf_it_30_kt_0.01_dm_0.8_points_opt.dat" lt rgb "#C2261E", "2023_10_01_15_39_51_conf_it_30_kt_0.01_dm_0.8_poses.dat" lt rgb "black", "2023_10_01_15_39_51_conf_it_30_kt_0.01_dm_0.8_poses_opt.dat" lt rgb "#C2261E"
#### Good aula magna *

#### Test 17
plot "2023_10_01_16_41_27_conf_it_30_kt_0.01_dm_0.3_points.dat" lt rgb "#57B8FF", "2023_10_01_16_41_27_conf_it_30_kt_0.01_dm_0.3_points_opt.dat" lt rgb "#C2261E", "2023_10_01_16_41_27_conf_it_30_kt_0.01_dm_0.3_poses.dat" lt rgb "#313638", "2023_10_01_16_41_27_conf_it_30_kt_0.01_dm_0.3_poses_opt.dat" lt rgb "#11D07D"


## Kernel 0.025
#### Test 21
plot "2023_09_30_22_58_00_conf_it_30_kt_0.025_dm_1000_points.dat" lt rgb "#57B8FF", "2023_09_30_22_58_00_conf_it_30_kt_0.025_dm_1000_points_opt.dat" lt rgb "#C2261E", "2023_09_30_22_58_00_conf_it_30_kt_0.025_dm_1000_poses.dat" lt rgb "#313638", "2023_09_30_22_58_00_conf_it_30_kt_0.025_dm_1000_poses_opt.dat" lt rgb "#11D07D"

#### Test 22
plot "2023_09_30_23_03_50_conf_it_30_kt_0.025_dm_100_points.dat" lt rgb "#57B8FF", "2023_09_30_23_03_50_conf_it_30_kt_0.025_dm_100_points_opt.dat" lt rgb "#C2261E", "2023_09_30_23_03_50_conf_it_30_kt_0.025_dm_100_poses.dat" lt rgb "#313638", "2023_09_30_23_03_50_conf_it_30_kt_0.025_dm_100_poses_opt.dat" lt rgb "#11D07D"
#### Test 23
plot "2023_09_30_23_18_25_conf_it_30_kt_0.025_dm_10_points.dat" lt rgb "#57B8FF", "2023_09_30_23_18_25_conf_it_30_kt_0.025_dm_10_points_opt.dat" lt rgb "#C2261E", "2023_09_30_23_18_25_conf_it_30_kt_0.025_dm_10_poses.dat" lt rgb "#313638", "2023_09_30_23_18_25_conf_it_30_kt_0.025_dm_10_poses_opt.dat" lt rgb "#11D07D"
#### Test 24
plot "2023_10_01_03_07_24_conf_it_30_kt_0.025_dm_1_points.dat" lt rgb "#57B8FF", "2023_10_01_03_07_24_conf_it_30_kt_0.025_dm_1_points_opt.dat" lt rgb "#C2261E", "2023_10_01_03_07_24_conf_it_30_kt_0.025_dm_1_poses.dat" lt rgb "#313638", "2023_10_01_03_07_24_conf_it_30_kt_0.025_dm_1_poses_opt.dat" lt rgb "#11D07D"
#### Test 25
plot "2023_10_01_03_15_20_conf_it_30_kt_0.025_dm_0.1_points.dat" lt rgb "black", "2023_10_01_03_15_20_conf_it_30_kt_0.025_dm_0.1_points_opt.dat" lt rgb "#C2261E", "2023_10_01_03_15_20_conf_it_30_kt_0.025_dm_0.1_poses.dat" lt rgb "black", "2023_10_01_03_15_20_conf_it_30_kt_0.025_dm_0.1_poses_opt.dat" lt rgb "#C2261E"
#### nice bottom right, problem in aula magna and bottom left
#### Test 26
plot "2023_10_01_03_19_59_conf_it_30_kt_0.025_dm_0.5_points.dat" lt rgb "#57B8FF", "2023_10_01_03_19_59_conf_it_30_kt_0.025_dm_0.5_points_opt.dat" lt rgb "#C2261E", "2023_10_01_03_19_59_conf_it_30_kt_0.025_dm_0.5_poses.dat" lt rgb "#313638", "2023_10_01_03_19_59_conf_it_30_kt_0.025_dm_0.5_poses_opt.dat" lt rgb "#11D07D"
#### Test 27
plot "2023_10_01_15_33_03_conf_it_30_kt_0.025_dm_0.8_points.dat" lt rgb "#57B8FF", "2023_10_01_15_33_03_conf_it_30_kt_0.025_dm_0.8_points_opt.dat" lt rgb "#C2261E", "2023_10_01_15_33_03_conf_it_30_kt_0.025_dm_0.8_poses.dat" lt rgb "#313638", "2023_10_01_15_33_03_conf_it_30_kt_0.025_dm_0.8_poses_opt.dat" lt rgb "#11D07D"

## Kernel 0.015
#### Test 31
plot "2023_10_01_15_33_03_conf_it_30_kt_0.025_dm_0.8_points.dat" lt rgb "#57B8FF", "2023_10_01_15_33_03_conf_it_30_kt_0.025_dm_0.8_points_opt.dat" lt rgb "#C2261E", "2023_10_01_15_33_03_conf_it_30_kt_0.025_dm_0.8_poses.dat" lt rgb "#313638", "2023_10_01_15_33_03_conf_it_30_kt_0.025_dm_0.8_poses_opt.dat" lt rgb "#11D07D"


## Kernel 0.0001
plot "2023_10_02_16_42_08_conf_it_30_kt_0.001_dm_0.1_points.dat" lt rgb "#57B8FF", "2023_10_02_16_42_08_conf_it_30_kt_0.001_dm_0.1_points_opt.dat" lt rgb "#C2261E", "2023_10_02_16_42_08_conf_it_30_kt_0.001_dm_0.1_poses.dat" lt rgb "#313638", "2023_10_02_16_42_08_conf_it_30_kt_0.001_dm_0.1_poses_opt.dat" lt rgb "#11D07D"


## min_50_corr
plot "2023_10_03_10_54_34_conf_it_30_kt_0.025_dm_0.1_points.dat" lt rgb "black", "2023_10_03_10_54_34_conf_it_30_kt_0.025_dm_0.1_points_opt.dat" lt rgb "#C2261E", "2023_10_03_10_54_34_conf_it_30_kt_0.025_dm_0.1_poses.dat" lt rgb "black", "2023_10_03_10_54_34_conf_it_30_kt_0.025_dm_0.1_poses_opt.dat" lt rgb "#C2261E"

## min 10 corr

plot "2023_10_03_10_59_46_conf_it_30_kt_0.025_dm_0.1_points.dat" lt rgb "black", "2023_10_03_10_59_46_conf_it_30_kt_0.025_dm_0.1_points_opt.dat" lt rgb "#C2261E", "2023_10_03_10_59_46_conf_it_30_kt_0.025_dm_0.1_poses.dat" lt rgb "black", "2023_10_03_10_59_46_conf_it_30_kt_0.025_dm_0.1_poses_opt.dat" lt rgb "#C2261E"

## good aula magna, good bottom right

plot "2023_10_03_13_23_32_conf_it_25_kt_0.025_dm_1_points.dat" lt rgb "black", "2023_10_03_13_23_32_conf_it_25_kt_0.025_dm_1_points_opt.dat" lt rgb "#C2261E", "2023_10_03_13_23_32_conf_it_25_kt_0.025_dm_1_poses.dat" lt rgb "black", "2023_10_03_13_23_32_conf_it_25_kt_0.025_dm_1_poses_opt.dat" lt rgb "#C2261E"


plot "2023_10_03_13_40_26_conf_it_30_kt_0.02_dm_0.1_points.dat" lt rgb "black", "2023_10_03_13_40_26_conf_it_30_kt_0.02_dm_0.1_points_opt.dat" lt rgb "#C2261E", "2023_10_03_13_40_26_conf_it_30_kt_0.02_dm_0.1_poses.dat" lt rgb "black", "2023_10_03_13_40_26_conf_it_30_kt_0.02_dm_0.1_poses_opt.dat" lt rgb "#C2261E"

### normal estimation methods

plot "2023_10_03_18_36_22_conf_it_30_kt_0.025_dm_0.1_points.dat" lt rgb "black", "2023_10_03_18_36_22_conf_it_30_kt_0.025_dm_0.1_points_opt.dat" lt rgb "#C2261E", "2023_10_03_18_36_22_conf_it_30_kt_0.025_dm_0.1_poses.dat" lt rgb "black", "2023_10_03_18_36_22_conf_it_30_kt_0.025_dm_0.1_poses_opt.dat" lt rgb "#C2261E"

### color palette

