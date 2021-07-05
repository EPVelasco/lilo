# Lilo
## LiLo: Lite 3D-Lidar Odometry

<p float="left">
  <img src="/images/test_loop.GIF" width="100" />
  <img src="/images/blue.jpg" width="100" /> 
</p>

This code obtains the odometry of a UGV ([BLUE](https://github.com/AUROVA-LAB/robot_blue) with the estimation of the position transform between two point-clouds (a current one with a previous one) obtained by a Velodyne VLP16 3D-lidar sensor.

This code is modified from [FLOAM](https://github.com/wh200720041/floam).

**Modifier:** Edison P. Velasco Sánchez, Universidad de Alicante, Spain.

## Requisites
- ROS kinetic or Melodic
- Velodyne repository
```
sudo apt-get install ros-$ROS_DYSTRO-velodyne-pointcloud
```
- PCL (Point Cloud Library)
- Ceres Solver
### Ceres instalation
- Clone the repository 
```
git clone https://ceres-solver.googlesource.com/ceres-solver
```
- Install all the dependencies
```
sudo apt-get install cmake
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev
```
- Build, test, and install Ceres.
```
tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.0.0
make -j3
make test
make install
```
## Test Lilo

<p align='center'>
<img width="80%" src="/images/Scientific Park.GIF"/>
</p>

## Launch
```
    roslaunch lilo lilo_velodyne.launch
```
