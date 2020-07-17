# LOAM-vehicle-filter
Removes vehicle points

## Prererequisites

* Ubuntu 18.04 with ROS Melodic
* Ceres Solver
* PCL

## Build

create and build catkin workspace

```

cd ~/aloam_ws/src
git clone https://github.com/lapman-0/LOAM_vehicle-filter
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash

```

## Run

```
roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch
rosbag play <YOUR_BAG.bag>

```
## Acknowledgements

Thanks to LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time)
