# tms_ss_terrain_static

tms_ss_terrain_static is a package for handling point cloud data of static terrain.

- tms_ss_terrain_static

  tms_ss_terrain_static converts PointCloud2 msg to .pcd file and sends the file info to [tms_db_writer_gridfs](../../tms_db/tms_db_manager).

- tms_ss_terrain_static_mesh

  tms_ss_terrain_static_mesh converts PointCloud2 msg to .ply (mesh) file and sends the file info to [tms_db_writer_gridfs](../../tms_db/tms_db_manager).

The received PointCloud2 msg is a point cloud data of static terrain.

# Usecase

## 1. Run tms_db_writer_gridfs or tms_db_manager

Detail description is [here](../../tms_db/tms_db_manager).

Receive TmsdbGridFS from tms_ss_terrain_static and store them in ROS2-TMS database.

### Writer

```
ros2 launch tms_db_manager tms_db_writer.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

### Reader and Writer

```
ros2 launch tms_db_manager tms_db_manager.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

## 2. Run tms_sd_terrain

Detail description is [here](../../tms_sd/tms_sd_terrain)

Relay static terrain's PointCloud2 msg to tms_ss_terrain_static.

```
ros2 launch tms_sd_terrain tms_sd_terrain_launch.py input/terrain/static/pointcloud2:=/topic/of/static/pointcloud2 input/terrain/dynamic/pointcloud2:=/topic/of/dynamic/pointcloud2
```

## 3. Run tms_ss_terrain_static

After the following command, a node is executed that subscribes PointCloud2 and publishes TmsdbGridFS including the .pcd or .ply file info converted from the PointCloud2.

```
ros2 launch tms_ss_terrain_static tms_ss_terrain_static_launch.py filename:=demo.pcd filename_mesh:=demo.ply voxel_size:=0.1 octree_depth:=8 density_th:=0.1
```

### Inputs / Outputs

**Inputs**

| Name                      | Type                            | Description                         |
| ------------------------- | ------------------------------- | ----------------------------------- |
| `/tms_sd_terrain_static`  | `sensor_msgs::msg::PointCloud2` | point cloud data of static terrain  |

**Outputs**

| Name                  | Type                           | Description                                                       |
| --------------------- | ------------------------------ | ----------------------------------------------------------------- |
| `/tms_db_gridfs_data` | `tms_msg_db::msg::TmsdbGridFS` | .pcd or .ply file info of static terrain or static terrain's mesh |

### Parameters

| Name            | Type   | Default Value       | Description                                                                |
| --------------- | ------ | ------------------- | -------------------------------------------------------------------------- |
| `filename`      | string | `filename.pcd`      | .pcd file name of static terrain converted from PointCloud2                |
| `filename_mesh` | string | `filename_mesh.ply` | .ply file name of static terrain's mesh converted from PointCloud2         |
| `voxel_size`    | float  | `0.0`               | voxel size of downsampling                                                 |
| `octree_depth`  | int    | `2`                 | octree depth to generate mesh                                              |
| `density_th`    | float  | `0.1`               | density threshold to remove vertices and triangles that have a low support |
