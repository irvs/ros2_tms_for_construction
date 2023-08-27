# tms_sp_machine

tms_sp_machine is a package for handling data realated to construction machines.

- tms_sp_machine_odom

  tms_sp_machine_odom converts Odometry msg to Tmsdb msg and send it to tms_db_writer.

  Received Odometry msg is the location data of the construction machine.

  Odometry msg topic is transformed by [machine_odom_tf_broadcaster node](../../tms_tf/tms_tf_gui/tms_tf_gui/machine_odom_tf_broadcaster.py).

- tms_sp_machine_points

  tms_sp_machine_points transforms the coordinates of the machine odom using tms_tf_gui.

  Since this package is used with tms_tf_gui, please refer to [this document](../../tms_tf/tms_tf_gui) for an example of using this package.

# Usecase

## 1. Run tms_db_writer or tms_db_manager

Detail description is [here](https://github.com/irvs/ros2_tms_for_construction/tree/main/tms_db).

Receive Tmsdb from tms_sp_machine and store them in ROS2-TMS database.

### Writer

```
ros2 launch tms_db_manager tms_db_writer.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

### Reader and Writer

```
ros2 launch tms_db_manager tms_db_manager.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

## 2. Run tms_sp_machine_odom

After the below command, a node is executed that subscribes Odometry and publishes Tmsdb including the Odometry.

```
ros2 launch tms_sp_machine tms_sp_machine_odom_launch.py input/odom:=/topic/of/odom machine_name:=crawler to_frame:=world
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
| `to_frame`     | string | `world`        | target frame_id                                               |
