# 2D Feature Tracking

![](/media/keypoints.png)

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.

2. Download the images:
    ```shell
    wget https://downgit.github.io/#/home?url=https://github.com/udacity/SFND_2D_Feature_Tracking/tree/master/images
    unzip images.zip
    ```
3. Make a build directory in the top level project directory: 
    ```shell
    mkdir build && cd build
    ```

4. Compile: 
    ```shell
    cmake .. && make
    ```

5. Run it: 
    ```shell
    ./2D_feature_tracking
    ```
