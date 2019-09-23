# tms_ss_whs1
## What is this
This is ROS2 version of ros_tms/tms_ss/tms_ss_whs1/src/main.cpp.
This saves the data of whs1 heartbeat sensor.

## How to use
After launching whs1_client(https://github.com/irvs/whs1_client/Debug/whs1_client.exe) on Windows PC,
and setting IPv4 address and port number of whs1_client and this program variable,

One terminal,

```
$ ros2 run tms_ss_whs1 tms_ss_whs1 
```

Another terminal,
```
$ ros2 run tms_db_manager tms_db_writer
```

### tms_ss_whs1_monitor
This is monitoring system of tms_ss_whs1 outputs.

One terminal,

```
$ ros2 run tms_ss_whs1 tms_ss_whs1_monitor
```

Another terminal,
```
$ ros2 run tms_db_manager tms_db_reader
```

### double_whs1_control
This node can control "double(robot)" by tms_ss_whs1.

IF your heartbeat over standard bpm, double moves.

One terminal,

```
$ ros2 run tms_ss_whs1 double_whs1_control
```

Another terminal,
```
$ ros2 run tms_db_manager tms_db_reader
```