# Lilo
## LiLo: Lite 3D-Lidar Odometry

<p float="left">
  <img src="/images/test_loop.GIF" width="410"  />
  <a href="https://github.com/AUROVA-LAB/robot_blue">
    <img src="/images/blue.jpg" width="380" /> 
  <a>
</p>

This code obtains the odometry of a UGV ([BLUE](https://github.com/AUROVA-LAB/robot_blue)) with the estimation of the position transform between two point-clouds (a current one with a previous one) obtained by a Velodyne VLP16 3D-lidar sensor.

This code is modified from [FLOAM](https://github.com/wh200720041/floam), which was based on [LOAM](https://github.com/laboshinl/loam_velodyne) and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) .
    
The contribution of this version of the code is that the point cloud map is not compiled and the location is generated using a configurable batch of data in the launch parameters. This means that odometry estimation times do not accumulate and are low.

**Modifier:** Edison P. Velasco Sánchez, Universidad de Alicante, Spain.

## Requisites
- [ROS](http://wiki.ros.org/ROS/Installation) Kinetic or Melodic
- [Velodyne](https://github.com/ros-drivers/velodyne) repository
  ```
  sudo apt-get install ros-melodic-velodyne-pointcloud
  ```
- [PCL](https://pointclouds.org/) (Point Cloud Library)
- Hector trajectory server
    ```
    sudo apt-get install ros-melodic-hector-trajectory-server
    ```
- Ceres Solver
    
### Ceres instalation
- Clone the repository 
```
git clone https://ceres-solver.googlesource.com/ceres-solver
```
- Install all the dependencies of Ceres
```
sudo apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev -y
```
- Build, test, and install Ceres. ([latest stable release](http://ceres-solver.org/installation.html))
```
tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.0.0
make -j3
sudo make install
```
## Clone repository
```
    cd ~/catkin_ws/src
    git clone https://github.com/EPVelasco/lilo.git
    cd ..
    catkin_make --only-pkg-with-deps lilo
```
  
                                          
## Ros Launch
```
    roslaunch velodyne_pointcloud VLP16_points.launch
    roslaunch lilo lilo_velodyne.launch
```
## Test 

<p align='center'>
<img width="80%" src="/images/Scientific Park.GIF"/>
</p>


