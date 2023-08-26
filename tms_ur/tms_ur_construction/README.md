# tms_ur_construction

tms_ur_construction is a package for getting construction data (ex. machine's location, terrain, hardness of ground) and publishing them.

# Usecase

## 1. Run tms_db_reader or tms_db_manager

Detail description is [here](https://github.com/irvs/ros2_tms_for_construction/tree/main/tms_db).

Receive requests from clients and return data in ROS2-TMS database to the clients.

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

After the below command, nodes are executed that request data to a service node and publish response data.

```
ros2 launch tms_ur_construction tms_ur_construction_launch.py output/occupancy_grid:=/topic/of/occupancy_grid latest:=true filename:=cloud.pcd voxel_size:=0.5
```

#### Services / Actions / Outputs

**Services**

| Name            | Type                            | Description                          |
| --------------- | ------------------------------- | ------------------------------------ |
| `tms_db_reader` | `tms_msg_db::srv::TmsdbGetData` | get data stored in ROS2-TMS database |

**Actions**

| Name                   | Type                              | Description                                 |
| ---------------------- | --------------------------------- | ------------------------------------------- |
| `tms_db_reader_gridfs` | `tms_msg_db::action::TmsdbGridFS` | get GridFS data stored in ROS2-TMS database |

**Outputs**

| Name                                  | Type                                | Description                                                 |
| ------------------------------------- | ----------------------------------- | ----------------------------------------------------------- |
| `/output/occupancy_grid`              | `nav_msgs::msg::OccupancyGrid`      | heatmap showing the hardness of the ground                  |
| `/output/terrain/static_srv`          | `tms_msg_db::srv::TerrainStaticSrv` | return point cloud data of static terrain to service client |
| `/output/terrain/dynamic/pointcloud2` | `sensor_msgs::msg::PointCloud2`     | point cloud data of dynamic terrain                         |
| `/output/odom`                        | `nav_msgs::msg::Odometry`           | location of machine                                         |

#### Parameters

| Name         | Type   | Default Value | Description                                |
| ------------ | ------ | ------------- | ------------------------------------------ |
| `latest`     | bool   | `False`       | whether to get the latest data             |
| `filename`   | string | `filename`    | .pcd file name stored in ROS2-TMS database |
| `voxel_size` | float  | `0.0`         | voxel size of downsampling                 |

### 2. construction_terrain_mesh

After the below command, a node is executed that requests mesh's .ply file info to a action service node and publishes Mesh converted from the file.

```
ros2 launch tms_ur_construction tms_ur_construction_terrain_mesh_launch.py output/terrain/mesh_srv:=/srv/of/mesh filename_mesh:=mesh.ply
```

#### Actions / Outputs

**Actions**

| Name                   | Type                              | Description                                 |
| ---------------------- | --------------------------------- | ------------------------------------------- |
| `tms_db_reader_gridfs` | `tms_msg_db::action::TmsdbGridFS` | get GridFS data stored in ROS2-TMS database |

**Outputs**

| Name                       | Type                              | Description                                          |
| -------------------------- | --------------------------------- | ---------------------------------------------------- |
| `/output/terrain/mesh_srv` | `tms_msg_db::srv::ColoredMeshSrv` | return mesh data of static terrain to service client |

#### Parameters

| Name         | Type   | Default Value | Description                                |
| ------------ | ------ | ------------- | ------------------------------------------ |
| `filename`   | string | `filename`    | .pcd file name stored in ROS2-TMS database |
| `voxel_size` | float  | `0.0`         | voxel size of downsampling                 |
| `alpha`      | float  | `1.0`         | alpha shapes of mesh                       |

### 3. construction_theta

After the below command, a node is executed that requests jpeg image data to a service node and publish response data.

```
ros2 launch tms_ur_construction tms_ur_construction_theta_launch.py output/theta/compressed:=/topic/of/compressed_image latest:=true theta_name:=theta
```

#### Services / Outputs

**Services**

| Name            | Type                            | Description                          |
| --------------- | ------------------------------- | ------------------------------------ |
| `tms_db_reader` | `tms_msg_db::srv::TmsdbGetData` | get data stored in ROS2-TMS database |

**Outputs**

| Name                       | Type                                | Description                              |
| -------------------------- | ----------------------------------- | ---------------------------------------- |
| `/output/theta/compressed` | `sensor_msgs::msg::CompressedImage` | a jpeg imag taken by a 360-degree camera |

#### Parameters

| Name         | Type   | Default Value | Description                                                          |
| ------------ | ------ | ------------- | -------------------------------------------------------------------- |
| `latest`     | bool   | `False`       | whether to get the latest data                                       |
| `theta_name` | string | `theta_name`  | 360-degree camera name to identify ground from the ROS2-TMS database |

### 4. ground_mesh

After the below command, a node is executed that requests ground and terrain mesh data to a service node and publish terrain mesh reflecting ground data.

```
ros2 launch tms_ur_construction tms_ur_construction_terrain_mesh_launch.py output/terrain/mesh_srv:=/srv/of/mesh filename_mesh:=mesh.ply
ros2 launch tms_ur_construction tms_ur_ground_mesh_launch.py output/ground_mesh:=/topic/of/mesh timer_period:=10
```

#### Services / Outputs

**Services**

| Name                       | Type                              | Description                                          |
| -------------------------- | --------------------------------- | ---------------------------------------------------- |
| `tms_db_reader`            | `tms_msg_db::srv::TmsdbGetData`   | get data stored in ROS2-TMS database                 |
| `/output/terrain/mesh_srv` | `tms_msg_db::srv::ColoredMeshSrv` | return mesh data of static terrain to service client |

**Outputs**

| Name                  | Type                           | Description                         |
| --------------------- | ------------------------------ | ----------------------------------- |
| `/output/ground_mesh` | `tms_msg_db::msg::ColoredMesh` | terrain mesh reflecting ground data |

#### Parameters

| Name           | Type   | Default Value  | Description              |
| -------------- | ------ | -------------- | ------------------------ |
| `timer_period` | string | `timer_period` | publisher's timer_period |
