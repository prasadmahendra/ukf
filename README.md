#### Unscented Kalman Filter

---

### Dependencies

* cmake >= 3.5
* All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
* Linux: make is installed by default on most Linux distros
* Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
* Linux: gcc / g++ is installed by default on most Linux distros
* Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
* Windows: recommend using [MinGW](http://www.mingw.org/)

### Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
* On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
some sample inputs in 'data/'.
- eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

### Build Instructions (xcode)

1. Clone this repo.
2. Make a build directory: `mkdir xcode-build && cd xcode-build`
3. Run: `cmake -G Xcode ../`
4. Open `UnscentedKF.xcodeproj` in xcode

### Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for Matlab scripts that can generate additional data.

### Results & Visualization of the output data
---
### Radar Only

**Input data 1**: data/input/sample-laser-radar-measurement-data-1.txt

![KF-Estimate vs. Measurement vs. Ground Truth](data/output/vis/UKF-Radar-Dataset-1.png "Sample data 1: KF-Estimate vs. Measurement vs. Ground Truth")

![NIS](data/output/vis/NIS-Radar-Dataset-1.png "Sample data 2: NIS")

| RMSE          |
| ------------- |
| 0.159048      |
| 0.188174      |
| 0.726899      |
| 0.675828      |

**Input data 2**: data/input/sample-laser-radar-measurement-data-2.txt

![KF-Estimate vs. Measurement vs. Ground Truth](data/output/vis/UKF-Radar-Dataset-2.png "Sample data 1: KF-Estimate vs. Measurement vs. Ground Truth")

![NIS](data/output/vis/NIS-Radar-Dataset-2.png "Sample data 2: NIS")

| RMSE          |
| ------------- |
| 0.785361      |
| 1.77988       |
| 2.35064       |
| 1.5312        |

### Lidar Only

**Input data 1**: data/input/sample-laser-radar-measurement-data-1.txt

![KF-Estimate vs. Measurement vs. Ground Truth](data/output/vis/UKF-Lidar-Dataset-1.png "Sample data 1: KF-Estimate vs. Measurement vs. Ground Truth")

![NIS](data/output/vis/NIS-Lidar-Dataset-1.png "Sample data 1: NIS")

| RMSE          |
| ------------- |
| 0.14044       |
| 0.123413      |
| 0.824945      |
| 0.685413      |

**Input data 2**: data/input/sample-laser-radar-measurement-data-2.txt

![KF-Estimate vs. Measurement vs. Ground Truth](data/output/vis/UKF-Lidar-Dataset-2.png "Sample data 1: KF-Estimate vs. Measurement vs. Ground Truth")

![NIS](data/output/vis/NIS-Lidar-Dataset-2.png "Sample data 2: NIS")

| RMSE          |
| ------------- |
| 0.215406      |
| 0.194694      |
| 0.543321      |
| 0.56718       |
                                            
### Radar & Lidar Combined

**Input data 1**: data/input/sample-laser-radar-measurement-data-1.txt

![KF-Estimate vs. Measurement vs. Ground Truth](data/output/vis/UKF-Both-Dataset-1.png "Sample data 1: KF-Estimate vs. Measurement vs. Ground Truth")

| RMSE          |
| ------------- |
| 0.0827558     |
| 0.0904532     |
| 0.657836      |
| 0.592343      |

**Input data 2**: data/input/sample-laser-radar-measurement-data-2.txt

![KF-Estimate vs. Measurement vs. Ground Truth](data/output/vis/UKF-Both-Dataset-2.png "Sample data 1: KF-Estimate vs. Measurement vs. Ground Truth")

| RMSE          |
| ------------- |
| 0.193222      |
| 0.188103      |
| 0.385123      |
| 0.511168      |


                                                                                                                                                          
