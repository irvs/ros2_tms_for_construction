# tms_db_manager

ROS2-TMS database manager.

# Usecase

## Reader

After the below command, service nodes are executed that get requested data in ROS2-TMS database and respond to them.

```
ros2 launch tms_db_manager tms_db_reader.launch.py db_host:=localhost db_port:=27017
```

### Servers

| Name                   | Type                                  | Description                                 |
| ---------------------- | ------------------------------------- | ------------------------------------------- |
| `tms_db_reader`        | `tms_msg_db::srv::TmsdbGetData`       | get data stored in ROS2-TMS database        |
| `tms_db_reader_gridfs` | `tms_msg_db::srv::TmsdbGridFSGetData` | get GridFS data stored in ROS2-TMS database |

## Writer

After the below command, subscriber nodes are executed that store received data in ROS2-TMS database.

```
ros2 launch tms_db_manager tms_db_writer.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

### Inputs

| Name                  | Type                           | Description                    |
| --------------------- | ------------------------------ | ------------------------------ |
| `/tms_db_data`        | `tms_msg_db::msg::Tmsdb`       | Tmsdb including other msg data |
| `/tms_db_gridfs_data` | `tms_msg_db::msg::TmsdbGridFS` | file info                      |

## Reader and Writer

Store data in ROS2-TMS database and get data from the database.

```
ros2 launch tms_db_manager tms_db_manager.launch.py db_host:=localhost db_port:=27017 init_db:=true
```

# Parameters

| Name      | Type   | Default Value | Description                        |
| --------- | ------ | ------------- | ---------------------------------- |
| `db_host` | string | `localhost`   | host name of MongoDB               |
| `db_port` | int    | `27017`       | port number of MongoDB             |
| `init_db` | bool   | `False`       | whether to initialize the database |
