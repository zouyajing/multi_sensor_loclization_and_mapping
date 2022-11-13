# multi_sensor_loclization_and_mapping
 It is the course work of [localization and mapping based on mutiple sensor fusion](https://www.shenlanxueyuan.com/course/558). Most of the codes are written by the tutors [Qian Ren] and [Yao Ge].
 
 It consists of :
 
 
   (a) a mapping node based on graph-based optimization, which utilze RTK, IMU, NDT-based Lidar odometry, and loop closure;
   
   (b) a localization node based on ESKF, which utlized IMU, NDT-based map matching, motion model and velocity;
   
   (c) a localization node based on graph optimization, which utlized IMU, NDT-based map matching, motion model and velocity;
 
 ## Dependencies
 
1. ROS
2. Eigen
3. PCL
4. g2o
5. ceres solver
6. Sophus
7. Protobuf
8. yaml
9. Geographic
10. Glog

I installed the same version as the third parties in [SLAMbook](https://github.com/gaoxiang12/slambook).

## Build

`catkin_make`

## Run

### Mapping

Run mapping pipeline to build a global map for the environment.

`roslaunch lidar_localization lio_mapping.launch`

### ESKF localization

Run localization pipeline based on ESKF using the built map

`roslaunch lidar_localization kitti_localization.launch`

### Graph-based localization
 Run localization pipeline based on graph optimization using the built map
 
`roslaunch lidar_localization lio_localization.launch`
