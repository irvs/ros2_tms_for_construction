# tms_ur_construction

tms_ur_construction is a package for getting construction data (ex. machine's location, terrain, hardness of ground) and publishing them.

# Usecase

## 1. Run tms_db_reader or tms_db_manager

Detail description is [here](https://github.com/irvs/ros2_tms_for_construction/tree/main/tms_db).

### Reader

```
ros2 launch tms_db_manager tms_db_reader.launch.py db_host:=localhost db_port:=27017
```

### Reader and Writer

```
ros2 launch tms_db_manager tms_db_manager.launch.py db_host:=localhost db_port:=27017 init_db:=false
```

## 2. Run tms_ur_construction

### 1. cv_odom, construction_ground, construction_terrain

```
ros2 launch tms_ur_construction tms_ur_construction_launch.py output/occupancy_grid:=/topic/of/occupancy_grid latest:=true file_name:=cloud.pcd voxel_size:=0.5
```

#### Servers / Outputs

**Servers**

| Name                   | Type                                  | Description                                 |
| ---------------------- | ------------------------------------- | ------------------------------------------- |
| `tms_db_reader`        | `tms_msg_db::srv::TmsdbGetData`       | get data stored in ROS2-TMS database        |
| `tms_db_reader_gridfs` | `tms_msg_db::srv::TmsdbGridFSGetData` | get GridFS data stored in ROS2-TMS database |

**Outputs**

| Name                     | Type                            | Description                                |
| ------------------------ | ------------------------------- | ------------------------------------------ |
| `/output/occupancy_grid` | `nav_msgs::msg::OccupancyGrid`  | heatmap showing the hardness of the ground |
| `/output/pointcloud2`    | `sensor_msgs::msg::PointCloud2` | point cloud data of terrain                |
| `/output/odom`           | `nav_msgs::msg::Odometry`       | location of machine                        |

#### Parameters

| Name         | Type   | Default Value | Description                                |
| ------------ | ------ | ------------- | ------------------------------------------ |
| `latest`     | bool   | `False`       | whether to get the latest data             |
| `filename`   | string | `filename`    | .pcd file name stored in ROS2-TMS database |
| `voxel_size` | float  | `0.0`         | voxel size of downsampling                 |

### 2. construction_terrain_mesh

```
ros2 launch tms_ur_construction tms_ur_construction_terrain_mesh_launch.py output/mesh:=/topic/of/mesh file_name:=cloud.pcd voxel_size:=0.5 alpha:=3.0
```

#### Servers / Outputs

**Servers**

| Name                   | Type                                  | Description                             |
| ---------------------- | ------------------------------------- | --------------------------------------- |
| `tms_db_reader_gridfs` | `tms_msg_db::srv::TmsdbGridFSGetData` | GridFS data stored in ROS2-TMS database |

**Outputs**

| Name           | Type                  | Description     |
| -------------- | --------------------- | --------------- |
| `/output/mesh` | shape_msgs::msg::Mesh | mesh of terrain |

#### Parameters

| Name         | Type   | Default Value | Description                                |
| ------------ | ------ | ------------- | ------------------------------------------ |
| `filename`   | string | `filename`    | .pcd file name stored in ROS2-TMS database |
| `voxel_size` | float  | `0.0`         | voxel size of downsampling                 |
| `alpha`      | float  | `1.0`         | alpha shapes of mesh                       |
