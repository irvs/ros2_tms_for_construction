# tms_mc_odom

tms_mc_odom is a package for formatting Odometry msg to Tmsdb msg and sending it to tms_db_writer.

Received Odometry msg is the location data of the construction machine.

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

## 2. Run tms_mc_odom

```
ros2 launch tms_mc_odom tms_mc_odom_launch.py input/odom:=/topic/of/odom machine_name:=crawler
```

### Inputs / Outputs

**Inputs**

| Name          | Type                      | Description         |
| ------------- | ------------------------- | ------------------- |
| `/input/odom` | `nav_msgs::msg::Odometry` | location of machine |

**Outputs**

| Name           | Type                     | Description              |
| -------------- | ------------------------ | ------------------------ |
| `/tms_db_data` | `tms_msg_db::msg::Tmsdb` | Tmsdb including Odometry |

### Parameters

| Name           | Type   | Default Value  | Description                                                   |
| -------------- | ------ | -------------- | ------------------------------------------------------------- |
| `machine_name` | string | `machine_name` | machine name to identify a machine from the ROS2-TMS database |
