# Lilo
## LiLo: Lite 3D-Lidar Odometry
This code obtains the odometry of a UGV ([BLUE](https://github.com/AUROVA-LAB/robot_blue) with the estimation of the position transform between two point-clouds (a current one with a previous one) obtained by a Velodyne VLP16 3D-lidar sensor.

This code is modified from [FLOAM](https://github.com/wh200720041/floam).

**Modifier:** Edison P. Velasco Sánchez, Universidad de Alicante, Spain.

## Requisites
- ROS kinetic or Melodic
- PCL (Point Cloud Library)
- Ceres Solver
### Ceres instalation

```
# clone the repository 
git clone https://ceres-solver.googlesource.com/ceres-solver
```

## Test
<p align='center'>
<img width="50%" src="/images/test_circuit.GIF"/>
</p>

## Launch
```
    roslaunch lilo lilo_velodyne.launch
```
