### 3. Store and get data simultaneously in real-time

Run the following commands to store data in MongoDB and get the data.

Static terrain data is not included because it does not need to be acquired in real-time.

#### Launch

```
# MongoDB manager
ros2 launch tms_db_manager tms_db_manager.launch.py

# Odometry
ros2 launch tms_sp_machine tms_sp_machine_odom_launch.py input/odom:=/demo2/odom machine_name:=demo_machine

# Ground 2D map
ros2 launch tms_sd_ground tms_sd_ground_launch.py input/occupancy_grid:=/demo2/map_2d ground_name:=demo_ground

# Terrain
ros2 launch tms_sd_terrain tms_sd_terrain_launch.py input/terrain/dynamic/pointcloud2:=/demo2/terrain/dynamic
```

#### Rviz2

```
rviz2 -d ./src/ros2_tms_for_construction/demo/demo2/demo2.rviz
```

#### Launch tms_ur_construction

Run the following commands to get data from MongoDB.

```
# Get static terrain
ros2 launch tms_ur_construction tms_ur_construction_terrain_static_launch.py filename:=demo.pcd voxel_size:=0.0

# Service Client and Publisher nodes for static terrain data
ros2 run tms_ur_test tms_ur_construction_terrain_static_test

# Get odometry, ground 2D map and terrain data
ros2 launch tms_ur_construction tms_ur_construction_launch.py machine_name:=demo_machine latest:=true
```

#### Play rosbag

```
ros2 bag play -l ./src/ros2_tms_for_construction/demo/demo2/rosbag2_2
```

Rviz will show odometry, ground 2D map and dynamic terrain data like a example of [2. Get stored data](#2-get-stored-data), excluding static terrain.

After the end of rosbag, please check whether the data is stored to fs.chunks, fs.files, machine and sensor collection in your MongoDB.

GUI tool of MongoDB like a MongoDB Compass is easy to check them.

Here is an example. It may be a little different than yours, but as long as it is roughly the same, you should be fine.

![](demo/demo2/demo_mongodb_compass.png)