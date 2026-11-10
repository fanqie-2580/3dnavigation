# DDDMR ODOM 3D
This package demonstrates a simple example of 3D odometry calculation and visualization for a differential-drive or skid-steer robot extended into 3D space. The goal is to help users understand how to use Euler angles and velocity for odometry calculation.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/odom_3d/Differential_drive_robot_3D_odometry_approximation_.png" width="520" height="620"/>
</p>

> [!CAUTION]
> - This is a 3D odometry example only. For production use, much finer considerations are required, including synchronization, data fusion, and robust filtering.
> - This example does not account for lateral (Y-axis) control input. It assumes no wheel sliping or lifting. It just integrates forward motion on X, while pitch affects the vertical component.
> - IMU orientation must be well-filtered. Raw noisy IMU data will lead to very inaccurate position estimates.

## Demo
This demo plays a bag with 2D odometry and IMU Euler angles to generate 3D odometry topic.
### 1. Create docker image
> [!NOTE]
> The package runs in the docker, so we need to build the image first. We support both x64 with or without GPU and arm64 (tested in nvidia jetson jpack6.2).
> 
> Follow the instruction to build either x64 or l4t docker images. It will take some time depending on your harware
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. The we can launch the demo in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
#### Play odom_3d_example using bag files in docker container
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch dddmr_odom_3d example_odom_3d_launch.py
```

The odom_3d_example node will fuse 2D odometry and IMU to 3D odometry. 
<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/odom_3d/3d_odom_demo.gif" width="700" height="420"/>
</p>

