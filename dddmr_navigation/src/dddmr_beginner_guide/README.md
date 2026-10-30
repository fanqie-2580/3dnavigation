# DDDMR BEGINNER GUIDE

This README is a beginner’s guide to the DDDMR Navigation Stack. With both a Gazebo quadruped robot example and a real robot guide, it’s designed to help you get up and running fast, explore, and have fun along the way.
## 🖥️ Software Requirements
- **Ubuntu 22.04** (tested in 22.04, should support 24.04)
- **Docker**  [install Docker](https://docs.docker.com/engine/install/)
## ✨ DDDMR Navigation with Gazebo
This demo demonstrates how to run the DDDMR Navigation Stack in Gazebo with a quadruped robot.
- Build the required images to prepare the environment.
- Run the system in two terminals — one for the Gazebo world and another for the navigation stack. 

### 1. Create docker image
Clone the repo and run ./build.bash, please select **`x64_gz`**, which already contains all the necessary components for both navigation and Gazebo.
```
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Download navigation map
To play gazebo with dddmr_navigation, you will need to download demo navigation map (12.3MB).
```
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Prepare demo enviroment
#### Create a two docker container (gazebo and navigaiton)
> [!IMPORTANT] 
> The following command will start two interactive Docker containers using the image we built.  Please open **two separate terminals** to prepare the demo environment

#### 🖥️ Terminal 1  (create the container for gazebo system)

- ##### Step 1 (on host): create the container for Gazebo system
```
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_gazebo.bash
```
- ##### Step 2 (inside the gazebo container): build and launch
```
source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch go2_config gz_lidar_odom.launch.py
```

#### 🖥️ Terminal 2   (create the container for navigation system)

   - ##### Step 1 (on host): create the container for navigation system
```
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```
   - ##### Step 2 (inside the navigation container): build and launch
```
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Run demo 
- In the Gazebo demo, the map is already aligned, so you don’t need to set the initial pose unless you are mapping yourself
- Give the goal (3D Goal Pose) in RViz , then the robot will move to target point

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Known Issues
> [!WARNING]
> The following are currently observed behaviors. They are under investigation and will be fixed in future updates.
 - Gazebo: Occasional slipping on slopes 
 - Mapping: Duplicate floor layers  



<summary><h2>💡 DDDMR Navigation with a Real Robot (Coming soon..)</h2></summary>
## ✨ Start DDDMR Navigation with a Real Robot
You should be able to run DDDMR Navigation like a charm if your system meet following requirements:

👉 Requirements: 
  1. Your robot can be controlled by a topic (/cmd_vel) based on geometry_msgs/msg/Twist type.
  2. You have a multi layer lidar and there is a ROS2 node publishing the point cloud based on sensor_msgs/msg/PointCloud2 type.
  3. You have a ROS2 node that publishes an odometry topic and tf based on nav_msgs/msg/Odometry type.
     
     * DDDMR Navigation should work if your quadruped robot, humanoid robot or wheel robot publish reasonable odometry topic/tf, i.e. error is less than 10%.
  5. When your system is launched, you should see following tf tree. In addition, make sure your the frame_id of your lidar topic is consistent with your tf tree:
      <p align='center'>
        <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="200" height="480"/>
      </p>
  6. If you do ros2 topic list, you should be able to see: /odom, /cmd_vel, /tf and /lidar_point_cloud
  7. You are good to go.

👉 Advanced:
  1. Implement [3D odometry](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) to get a better and robust localization and mapping results.

## 🚧 Start Mapping
There are 2️⃣ mapping approaches supported in dddmr_navigation.

### Mapping from a Bag File - Offline Mapping
To map the area, you can record two ROS2 topics for the offline mapping. Recording odom topic and point cloud topic while manually drive your robot in the area.
Odom topic for the offline mapping is not mandatory, but in some cases such as featureless environment or wide open area, it can be used to improve the mapping quality.

```
ros2 bag record /odom /lidar_point_cloud
```

Once you have the bag file, modify your directory, pointcoud topic and odom topic in the off-line launch file:
https://github.com/dfl-rlab/dddmr_navigation/blob/7706c3333aa9dbc90a4c18598cef05d39388052f/src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_c16_config.yaml#L4
In addition, make sure the lidar spec is correctly setup:
https://github.com/dfl-rlab/dddmr_navigation/blob/7706c3333aa9dbc90a4c18598cef05d39388052f/src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_c16_config.yaml#L13

And then you can run the offline mapping by:

```
ros2 launch lego_loam_bor lego_loam_bag.launch
```
### Mapping in Realtime - Online Mapping
Similar to the offline mapping, setup the lidar spec correctly in the configutation file:
https://github.com/dfl-rlab/dddmr_navigation/blob/7706c3333aa9dbc90a4c18598cef05d39388052f/src/dddmr_lego_loam/lego_loam_bor/config/loam_c16_config.yaml#L4
Change the corresponding topics at:
https://github.com/dfl-rlab/dddmr_navigation/blob/7706c3333aa9dbc90a4c18598cef05d39388052f/src/dddmr_lego_loam/lego_loam_bor/launch/lego_loam.launch#L6

And then you can run the online mapping by:
```
ros2 launch lego_loam_bor lego_loam.launch
```
## 🚧 Start Localization

## 🚧 Start Point to Point Navigation
