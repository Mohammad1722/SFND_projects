# Lidar Obstacle Detection

![](/media/ObstacleDetectionFPS.gif)

## Local Installation

### Ubuntu 


1. Install [Point Cloud Library (PCL)](https://pointclouds.org/):
    ```bash
    sudo apt install libpcl-dev
    ```

2. Clone this github repo

3. Open a terminal in the local repo folder and execute the following commands
   ```shell
   cd src/sensors/data/pcd
   wget https://downgit.github.io/#/home?url=https://github.com/udacity/SFND_Lidar_Obstacle_Detection/tree/master/src/sensors/data/pcd/data_1
   wget https://downgit.github.io/#/home?url=https://github.com/udacity/SFND_Lidar_Obstacle_Detection/tree/master/src/sensors/data/pcd/data_2
   unzip data_1.zip
   unzip data_2.zip
   cd ../../../..
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```
