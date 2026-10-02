### 2. Try running the task schedular with OperaSim-PhysX

This chapter explain how to link ROS2-TMS for Construction and OperaSim-PhysX , which is being developed by PWRI as a simulator of OPERA.

Please follow the instructions described in the ReadMe on the official GitHub page of OperaSim-PhysX (URL: https://github.com/pwri-opera/OperaSim-PhysX) on how to set up windows PC and ubuntu 22.04 PC for using OperaSimPhysX.

Once the connection between OperaSim-PhysX and ROS2 Humble is established, run the following command to start ROS2-TMS-for-construction on Ubuntu22.04 PC.

```
cd ~/ros2-tms-for-construction_ws
source install/setup.bash
ros2 launch tms_ts_launch tms_ts_construction.launch.py
```

As explained in Chapter 1, you can execute the specified task using the task scheduler by clicking the green button that appears when starting ros2-tms-for-construction. If you want to make an emergency stop while executing a task, click on the red button.



Additionally, the current ROS2-TMS for Construction includes several tasks for operating actual construction machinery and machines on OperaSim-PhysX. 
The summary of the task data currently stored in the database is as follows:


| task_id | The contents of the task | Used machine |
| ------------ | -------- | ---- |
| 1 | Excavate and load the soil once using the zx200 | zx200(MoveIt!) |
| 2 | Excavate and load the soil four times using the zx200 | zx200(MoveIt!) |
| 3 | The ic120 navigates along the route between two points | ic120(Nav2) |
| 4 | The ic120 makes two round trips between the loading point and the dumping point | ic120(Nav2) |
| 5 | Excavate and load the soil using the zx200, then transport it with the ic120 (2 trips) | zx200(MoveIt!) and ic120(Nav2) |


Additionally, to successfully execute the tasks in the table above, it is necessary to pre-launch the ROS2 packages for zx200 and ic120 prepared on the OPERA. Because the packages to launch differ for cases involving the operation of zx200 and ic120, the procedures are explained separately below.


#### Packages for operating OPERA-compatible IC120 on the OperaSim-PhysX using Nav2! and MoveIt! (task_id: )


#### Step1 
Launch the ROS-TCP-Endpoint and start communication between Unity and ROS 2.
```
# Open the 1st terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```
***
***

#### Step2
Select and start the appropriate planner for the construction equipment's autonomous operation, as required. Enter it into another terminal.

***
**related for excavator(zx200)**

**zx200(MoveIt!)** - Perform manipulation of the zx200, swing boom, arm and bucket.
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch zx200_bringup vehicle.launch.py command_interface_name:=velocity use_rviz:=true
```
**zx200(Nav2)** - Execute navigation for the zx200
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch zx200_bringup remote_navigation.launch.py
```

***
**related for crawler dump truck(ic120)**

**ic120(Nav2)** - Execute navigation for the ic120
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch ic120_unity ic120_standby_ekf.launch.py
```

***
**related for crawler dump truck(mst110cr)**

**mst110cr(Nav2)** - Execute navigation for the mst110cr
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch mst110cr_unity mst110cr_standby_ekf.launch.py robot_name:=<machine name> 
```


**mst110cr(swing and vessel angle for OperaSim)** - Perform manipulation of the mst110cr, swing and vessel. **Not for real machine.**

```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch opera_tools opera_tools_crawlerdump.launch.py robot_name:=<machine name> 
```
***
**related for blldozer(d37pxi)**

**d37pxi(Nav2)** - Execute navigation for the d37pxi
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch d37pxi_unity d37pxi_standby_ekf.launch.py robot_name:=<machine name>
```

**d37pxi(blade angle for OperaSim)** - Perform manipulation of the d37pxi blade. **Not for real machine.**
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch opera_tools opera_tools_bulldozer.launch.py robot_name:=<machine name> 
```

***
***

#### Step3
Establish a connection between ROS2-TMS for Construction and OPERA
```
# Open the 3rd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_if_for_opera tms_if_for_opera.launch.py
```
***
To start a node for each construction machine, run the following command on each machine.
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_if_for_opera tms_if_for_opera_excavator.launch.py robot_name:=<machine name>
```
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_if_for_opera tms_if_for_opera_crawlerdump.launch.py robot_name:=<machine name>
```
```
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_if_for_opera tms_if_for_opera_bulldozer.launch.py robot_name:=<machine name>
```

***
***

#### Step4
Launch the construction equipment operation task and start autonomous operation.
```
# Open the 4th terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_ts_launch tms_ts_construction.launch.py task_id:=<task_id>
```
----
----




Of course, you can also use Groot to monitor the tasks being performed by the Behavior Tree while the Task Scheduler is running, as shown in the following video.


https://github.com/irvs/ros2_tms_for_construction/assets/130209264/8747df87-0dd9-42c4-9132-6454c15eeedf
