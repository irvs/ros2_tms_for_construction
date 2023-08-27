# tms_sd_theta

tms_sd_theta is a package for formatting CompressedImage msg to Tmsdb msg and sending it to tms_db_writer.

Received CompressedImage msg is the jpeg image taken by a 360-degree camera.

# Usecase

## 1. Run tms_db_writer or tms_db_manager

Detail description is [here](https://github.com/irvs/ros2_tms_for_construction/tree/main/tms_db).

Receive Tmsdb from tms_sp_machine_odom and store them in ROS2-TMS database.

### Writer

```
ros2 launch tms_db_manager tms_db_writer.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

### Reader and Writer

```
ros2 launch tms_db_manager tms_db_manager.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

## 2. Run tms_sd_theta

After running the following command, a node is executed that subscribes CompressedImage and publishes Tmsdb including the CompressedImage data.

```
ros2 launch tms_sd_theta tms_sd_theta_launch.py input/theta/compressed:=/topic/of/compressed_image theta_name:=theta_name
```

### Inputs / Outputs

**Inputs**

| Name                      | Type                                | Description                               |
| ------------------------- | ----------------------------------- | ----------------------------------------- |
| `/input/theta/compressed` | `sensor_msgs::msg::CompressedImage` | a jpeg image taken by a 360-degree camera |

**Outputs**

| Name           | Type                     | Description                     |
| -------------- | ------------------------ | ------------------------------- |
| `/tms_db_data` | `tms_msg_db::msg::Tmsdb` | Tmsdb including CompressedImage |

### Parameters

| Name         | Type   | Default Value | Description                                                          |
| ------------ | ------ | ------------- | -------------------------------------------------------------------- |
| `theta_name` | string | `theta_name`  | 360-degree camera name to identify ground from the ROS2-TMS database |