# FAST-LIO-MULTI_ROS2

This repository is a **ROS2 port** of [FAST-LIO-MULTI](https://github.com/engcang/FAST_LIO_MULTI),  
which itself is a multi-LiDAR extension of [FAST-LIO2](https://github.com/hku-mars/FAST_LIO).

- Original project: **FAST-LIO-MULTI** (ROS1)
- This fork: **FAST-LIO-MULTI_ROS2** (ROS2)
- ROS2 integration and structure inspired by: [FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)

Optionally, the user can choose one of:
- **Bundle update**
- **Asynchronous update**
- **Adaptive update**

The algorithmic behavior of the three update modes is kept as close as possible to the original ROS1 implementation, with only the ROS interface and required glue adapted to ROS2.

---

## Related video

Original FAST-LIO-MULTI demo video (ROS1):  
https://youtu.be/YQmjKMoBPNU

The update methods and performance characteristics described there also apply conceptually to this ROS2 port.

---

## Dependencies

### Core

- **ROS2** (tested with Humble)
- **Ubuntu** (tested with 22.04)
- **PCL** ≥ 1.8  
- **Eigen** ≥ 3.3.4

### Livox driver (ROS2)

For Livox sensors, use the ROS2 driver:

- [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2)

Example installation:

```bash
# In your ROS2 workspace
cd ~/your_ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## How to build and run (ROS2)

### 1. Get the code

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/Draxran/FAST_LIO_MULTI_ROS2.git
cd FAST_LIO_MULTI_ROS2
git submodule update --init --recursive
```

### 2. Build with `colcon`

From the workspace root:

```bash
cd ~/your_ros2_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 3. Run

A ROS2 launch file is provided:

```bash
# Bundle update
ros2 launch fast_lio_multi fast_lio_multi.launch.py update_method:=bundle

# Asynchronous update
ros2 launch fast_lio_multi fast_lio_multi.launch.py update_method:=async

# Adaptive update
ros2 launch fast_lio_multi fast_lio_multi.launch.py update_method:=adaptive
```

The launch file wraps the same three mapping nodes (bundle / async / adaptive) as in the original ROS1 project, but using ROS2 nodes, topics, and parameters.

---

<br>

## Update methods: bundle vs asynchronous vs adaptive
+ Bundle update: merge multi LiDAR scans into one pointcloud, and then update
	+ Prevent no scan data input in extreme situation, e.g., high altitude flight of drones
	+ Longer update interval (which may cause drift during aggresive and fast movement from state propagation with only IMU)
	+ **NOTE: current code implementation will properly work for LiDARs with same scan rates (e.g., same 10Hz)**
+ Asynchronous update: update the filter whenever LiDAR scan inputs
	+ Shorter update interval (which may reduce drift from state propagation with only IMU)
	+ Depending on the sensor configuration, none-scanned data update may occur (which may result in divergence)
+ Adaptive update method
  + Asynchronous update => bundle update (only when data in FoV is not enough) => asynchronous update
  + Shorter update interval + preventing no scan data input!

<p align="center">
  <img src="imgs/bundle_method.png" width="800"/>
  <img src="imgs/async.png" width="800"/>
  <img src="imgs/adaptive.png" width="800"/>
  <br>
  <em>Update methods - (upper): Bundle, (middle): Asynchronous, (bottom): Adaptive</em>
</p>

+ By utilizing the forward and backward propagation structure of FAST-LIO2, each update method is implemented as follows:
<p align="center">
  <img src="imgs/updates.png" width="800"/>
  <br>
  <em>Update methods - (left): Bundle (right): Asynchronous</em>
</p>

<br>

## Results of each method (for better understanding, please watch the [related video](https://youtu.be/YQmjKMoBPNU))
+ For two sensor configurations,
	+ Config1: Livox-MID360 x 2EA (each is tilted +143, -143 degree)
	+ Config2: Livox-MID360 x 1EA (0 degree tilted), Livox-AVIA x 1EA (90 degree tilted)

<p align="center">
  <img src="imgs/config1.png" width="300"/>
  <img src="imgs/config2.png" width="300"/>
  <br>
  <em>Sensor config - (left): config1 (right): config2</em>
</p>

+ For aggresive motion and middle-altitude flight situation with sensor config1, asynchronous update method shows better performance
	+ Green: ground-truth, turquoise: FAST-LIO-MULTI
<p align="center">
  <img src="imgs/bundle.png" width="400"/>
  <img src="imgs/naive.png" width="400"/>
  <br>
  <em>Side view - (left): Bundle (right): Async</em>
</p>
<p align="center">
  <img src="imgs/bundle2.png" width="400"/>
  <img src="imgs/naive2.png" width="400"/>
  <br>
  <em>Top view - (left): Bundle (right): Async</em>
</p>

+ For high-altitude flight situation (no many scanned data) with sensor config2, bundle update method shows better and robust performance
	+ Green: ground-truth, turquoise: FAST-LIO-MULTI
<p align="center">
  <img src="imgs/bundle_sparse.png" width="400"/>
  <img src="imgs/naive_sparse.png" width="400"/>
  <br>
  <em>Top view - (left): Bundle (right): Async</em>
</p>
<p align="center">
  <img src="imgs/bundle_sparse_side.png" width="400"/>
  <img src="imgs/naive_sparse_side.png" width="400"/>
  <br>
  <em>Side view - (left): Bundle (right): Async</em>
</p>
