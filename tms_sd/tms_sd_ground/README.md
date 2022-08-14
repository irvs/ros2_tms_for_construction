# tms_sd_ground

tms_sd_ground is a package for formatting OccupancyGrid msg to Tmsdb msg and sending it to tms_db_writer.

Received OccupancyGrid msg is a heatmap showing the hardness of the ground.

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

## 2. Run tms_sd_ground

```
ros2 launch tms_sd_ground tms_sd_ground_launch.py input/occupancy_grid:=/topic/of/occupancy_grid ground_name:=construction_ground
```

### Inputs / Outputs

**Inputs**

| Name                    | Type                           | Description                                |
| ----------------------- | ------------------------------ | ------------------------------------------ |
| `/input/occupancy_grid` | `nav_msgs::msg::OccupancyGrid` | heatmap showing the hardness of the ground |

**Outputs**

| Name           | Type                     | Description                   |
| -------------- | ------------------------ | ----------------------------- |
| `/tms_db_data` | `tms_msg_db::msg::Tmsdb` | Tmsdb including OccupancyGrid |

### Parameters

| Name                   | Type   | Default Value            | Description                                               |
| ---------------------- | ------ | ------------------------ | --------------------------------------------------------- |
| `ground_name`          | string | `ground_name`            | ground name to identify ground from the ROS2-TMS database |
