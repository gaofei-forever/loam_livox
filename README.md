# LOAM-Livox
## A robust LiDAR Odometry and Mapping (LOAM) package for Livox-LiDAR

<div align="center">
    <img src="pics/zym_rotate.gif" width = 45% >
    <img src="pics/hkust_stair.gif" width = 45% >
</div>

**Loam-Livox** is a robust, low drift, and real time odometry and mapping package for [*Livox LiDARs*](https://www.livoxtech.com/), significant low cost and high performance LiDARs that are designed for massive industrials uses. Our package address many key issues: feature extraction and selection in a very limited FOV, robust outliers rejection, moving objects filtering, and motion distortion compensation. In addition, we also integrate other features like parallelable pipeline, point cloud management using cells and maps, loop closure, utilities for maps saving and reload, etc. To know more about the details, please refer to our related paper:)

<div align="center">
    <img src="pics/loop_4in1.png" width = 100% >
</div>


In the development of our package, we reference to LOAM, [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED), and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

**Developer:** [Jiarong Lin](https://github.com/ziv-lin)

**Our related paper**: our related papers are now available on arxiv:
1. [Loam_livox: A fast, robust, high-precision LiDAR odometry and mapping package for LiDARs of small FoV](https://arxiv.org/abs/1909.06700)
2. [A fast, complete, point cloud based loop closure for LiDAR odometry and mapping](https://arxiv.org/abs/1909.11811)

**Our related video**: our related videos are now available on YouTube (click below images to open):
<div align="center">
<a href="https://youtu.be/WHbbtU-Q9-k" target="_blank"><img src="pics/video_loam.png" alt="video" width="45%" /></a>
<a href="https://youtu.be/Uq8rUEk-XnI" target="_blank"><img src="pics/video_lc.png" alt="video" width="45%" /></a>
</div>


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation) and its additional ROS pacakge:

```
    sudo apt-get install ros-XXX-cv-bridge ros-XXX-tf ros-XXX-message-filters ros-XXX-image-transport
```
**NOTICE:** remember to replace "XXX" on above command as your ROS distributions, for example, if your use ROS-kinetic, the command should be:

```
    sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport
```

### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

**NOTICE:** Recently, we find that the point cloud output form the voxelgrid filter vary form PCL 1.7 and 1.9, and PCL 1.7 leads some failure in some of our examples ([issue #28](https://github.com/hku-mars/loam_livox/issues/28)). By this, we strongly recommand you to use update your PCL as version 1.9 if you are using the lower version.

## 2. Build
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/hku-mars/loam_livox.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3. Directly run
### 3.1 Livox Mid-40
Connect to your PC to Livox LiDAR (Mid-40) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then (launch our algorithm **first**, then livox-ros-driver):

```
    roslaunch loam_livox livox.launch
    roslaunch livox_ros_driver livox_lidar.launch
```

## 4. Rosbag Example
### 4.1. **Common rosbag**
Download [Our recorded rosbag](https://drive.google.com/drive/folders/1HWomWWPSEVvka2QVB2G41iRvSwIt5NWf?usp=sharing) and then
```
roslaunch loam_livox rosbag.launch
rosbag play YOUR_DOWNLOADED.bag
```
<div align="center">
    <img src="pics/CYT_01.png" width=45% >
    <img src="pics/CYT_02.png" width=45% >
</div>

<div align="center">
    <img src="pics/HKU_ZYM_01.png" width=45% >
    <img src="pics/HKU_ZYM_02.png" width=45% >
</div>



