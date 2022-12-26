# tms_sd_terrain

tms_sd_tarrain package has two nodes tms_sd_terrain_static and tms_sd_terrain_dynamic.

- tms_sd_terrain_static
  
  tms_sd_terrain_static converts PointCloud2 msg to .pcd file and sends the file info to tms_db_writer_gridfs.

- tms_sd_terrain_dynamic

  tms_sd_terrain_dynamic formats PointCloud2 msg to TmsdbGridFS msg and sends it to tms_db_writer_gridfs.

Received PointCloud2 msg is a point cloud data of terrain.

# Usecase

## 1. Run tms_db_writer or tms_db_manager

Detail description is [here](https://github.com/irvs/ros2_tms_for_construction/tree/main/tms_db).

Receive TmsdbGridFS from tms_sd_terrain and store them in ROS2-TMS database.

### Writer

```
ros2 launch tms_db_manager tms_db_writer.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

### Reader and Writer

```
ros2 launch tms_db_manager tms_db_manager.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

## 2. Run tms_sd_terrain

After the below command, a node is executed that subscribes PointCloud2 and publishes TmsdbGridFS including the .pcd file info converted from the PointCloud2.

```
ros2 launch tms_sd_terrain tms_sd_terrain_launch.py input/terrain/static/pointcloud2:=/topic/of/static/pointcloud2 input/terrain/dynamic/pointcloud2:=/topic/of/dynamic/pointcloud2 filename:=demo.pcd
```

### Inputs / Outputs

**Inputs**

| Name                                 | Type                            | Description                         |
| ------------------------------------ | ------------------------------- | ----------------------------------- |
| `/input/terrain/static/pointcloud2`  | `sensor_msgs::msg::PointCloud2` | point cloud data of static terrain  |
| `/input/terrain/dynamic/pointcloud2` | `sensor_msgs::msg::PointCloud2` | point cloud data of dynamic terrain |

**Outputs**

| Name                  | Type                           | Description                                                             |
| --------------------- | ------------------------------ | ----------------------------------------------------------------------- |
| `/tms_db_gridfs_data` | `tms_msg_db::msg::TmsdbGridFS` | .pcd file info of static terrain or point cloud data of dynamic terrain |

### Parameters

| Name                | Type   | Default Value         | Description                                                 |
| ------------------- | ------ | --------------------- | ----------------------------------------------------------- |
| `filename`          | string | `filename`            | .pcd file name of static terrain converted from PointCloud2 |
