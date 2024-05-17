### 1. Store data

At first, you need to store static terrain data in MongoDB with running the following commands.

#### Launch

Please run the following commands in separate terminals.

```
# MongoDB manager
ros2 launch tms_db_manager tms_db_writer.launch.py init_db:=true

# Terrain
ros2 launch tms_sd_terrain tms_sd_terrain_launch.py input/terrain/static/pointcloud2:=/demo2/terrain/static

# Static terrain
ros2 launch tms_ss_terrain_static tms_ss_terrain_static_launch.py filename:=demo.pcd filename_mesh:=demo.ply filename_dem:=demo.npy voxel_size:=0.1 octree_depth:=8 density_th:=0.1 fill_nan_type:=avg resolution:=0.1
```

#### Play rosbag

```
ros2 bag play -l ./src/ros2_tms_for_construction/demo/demo2/rosbag2_1
```

If the following message appears on the terminal where tms_sd_terrain is executed, stop the terminal playing rosbag.

```
Static terrain info was received!
```

Then, run the following commands to store other data in MongoDB.

#### Launch

```
# Odometry
ros2 launch tms_sp_machine tms_sp_machine_odom_launch.py input/odom:=/demo2/odom machine_name:=demo_machine

# Ground 2D map
ros2 launch tms_sd_ground tms_sd_ground_launch.py input/occupancy_grid:=/demo2/map_2d ground_name:=demo_ground

# Terrain
ros2 launch tms_sd_terrain tms_sd_terrain_launch.py input/terrain/dynamic/pointcloud2:=/demo2/terrain/dynamic
```

#### Play rosbag

```
ros2 bag play ./src/ros2_tms_for_construction/demo/demo2/rosbag2_2
```

After the end of rosbag, please check whether the data is stored to fs.chunks, fs.files, machine and sensor collection in your MongoDB.

GUI tool of MongoDB like a MongoDB Compass is easy to check them.

Here is an example. It may be a little different than yours, but as long as it is roughly the same, you should be fine.

![](demo/demo2/demo_mongodb_compass.png)
