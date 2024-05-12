### 2. Get stored data

Run the following commands to get data from MongoDB.

#### Launch

```
# MongoDB manager
ros2 launch tms_db_manager tms_db_reader.launch.py

# Get static terrain
ros2 launch tms_ur_construction tms_ur_construction_terrain_static_launch.py filename:=demo.pcd voxel_size:=0.0

# Service Client and Publisher node for static terrain data
ros2 run tms_ur_test tms_ur_construction_terrain_static_test

# Get odometry, ground 2D map and terrain data
ros2 launch tms_ur_construction tms_ur_construction_launch.py machine_name:=demo_machine
```

#### Rviz2

```
rviz2 -d ./src/ros2_tms_for_construction/demo/demo2/demo2.rviz
```

Rviz will show odometry, ground 2D map and terrain data.

Since static terrain data is large, it's may be better to stop the terminal execting tms_ur_test if static terrain is displayed on Rviz.

https://user-images.githubusercontent.com/63947554/220530548-b2e1c23c-2938-4b8f-b8f7-60b93b46f702.mp4