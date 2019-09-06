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