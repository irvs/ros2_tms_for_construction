# tms_sd_terrain

tms_sd_terrain is a package for converting PointCloud2 msg to .pcd file and sending the file info to tms_db_writer_gridfs.

Received PointCloud2 msg is a point cloud data of terrain.

# Usecase

## 1. Run tms_db_writer or tms_db_manager

Detail description is [here](https://github.com/irvs/ros2_tms_for_construction/tree/main/tms_db).

### Writer

```
ros2 launch tms_db_manager tms_db_writer.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

### Reader and Writer

```
ros2 launch tms_db_manager tms_db_manager.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

## 2. Run tms_sd_terrain

```
ros2 launch tms_sd_terrain tms_sd_terrain_launch.py input/pointcloud2:=/topic/of/pointcloud2 filename:=cloud.pcd
```

### Inputs / Outputs

**Inputs**

| Name                 | Type                            | Description                 |
| -------------------- | ------------------------------- | --------------------------- |
| `/input/pointcloud2` | `sensor_msgs::msg::PointCloud2` | point cloud data of terrain |

**Outputs**

| Name                  | Type                           | Description    |
| --------------------- | ------------------------------ | -------------- |
| `/tms_db_gridfs_data` | `tms_msg_db::msg::TmsdbGridFS` | .pcd file info |

### Parameters

| Name                | Type   | Default Value         | Description                               |
| ------------------- | ------ | --------------------- | ----------------------------------------- |
| `filename`          | string | `filename`            | .pcd file name converted from PointCloud2 |
