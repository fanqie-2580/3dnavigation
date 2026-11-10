# dddmr_semantic_segmentation

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_semantic_segmentation/dddmr_semantic_segmentation_to_pointcloud.gif" width="640" height="440"/><p align='center'>Semantic segmentation and corrensponding point cloud</p>
</p>

## Pipeline

DDDMR semantic segmentation convert semantic segmentation result and align it with depth image to colorize the point cloud.
The TensorRT is used to faciliate the inference speed, we can achieve ~15 fps of resulted point cloud.

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_semantic_segmentation/dddmr%20semantic%20segmentation.png" width="780" height="560"/>
</p>

## Performace in the platform

| Platform (R36.4) with one Realsense D455 on                                 | fps   | Average CPU   | Average GPU  |
|-----------------------------------------------------------------------------|-------|---------------|--------------|
| Jetson Orin Nano                                                            | 15    | 58.0%         | 75.0%        |
| Jetson Orin AGX 32GB                                                        | 19    | 24.0%         | 30.0%        |

## Demo Tutorial

Request:
This pipeline requires RGBD camera.

#### 1. Build the docker image
```
cd ~/dddmr_navigation/dddmr_docker/docker_file/
./build.bash
```
Follow the instructions in the terminal, in my case, I type in l4t to build docker image for my Jetson Orin.

#### 2. Create a docker container
```
cd ~/dddmr_navigation/dddmr_docker/
./run_demo.bash
```

#### 3. Generate TensorRT file from the onnx:
From Step 1 and 2, you will enter into the docker container, in the container you should be able to convert the [DDRNet](https://github.com/ydhongHIT/DDRNet) onnx to TensorRT engine.
```
cd /root/dddmr_navigation/src/dddmr_semantic_segmentation/model
/usr/src/tensorrt/bin/trtexec --onnx=ddrnet_23_slim_dualresnet_citys_best_model_424x848.onnx --saveEngine=ddrnet_23_slim_dualresnet_citys_best_model_424x848.trt
```

#### 4. Compile and Run
```
cd /root/dddmr_navigation
source /opt/ros/humble/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch dddmr_semantic_segmentation rs_semantic_segmentaton_trt_launch.py
```
#### 5. Run exclusive point cloud segmentation
Download bag file for example, in your host (not in docker!)
```
cd ~/dddmr_navigation/src/dddmr_semantic_segmentation/
./download_files.bash
```
In your docker
```
ros2 launch dddmr_semantic_segmentation bag_exclude_ss_trt_launch.py
```
The class 0 is excluded from pointcloud. Class number can be checked here:

https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_semantic_segmentation/data/colors_mapillary.csv

For example, sidewalk is 0, parking is 1.
