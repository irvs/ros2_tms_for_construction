### 2. Make motion parameters from "OperaSimVR"
#### This chapter explains how to create parameters used for autonomous construction. If you don't use "OperaSimVR" or this writing function,  you can skip this chapter.   


#
#### Vizualizing location of machine by Rviz2 or OperaSimVR
To read the position and joint angles of a construction vehicle stored in mongoDB using the method in Chapter 1 and visualize them in OperaSimVR, run the following steps:

Please rewrite "executable" and output topic neme, machine name to your system in "tms_ur_cv_odom_demo_launch.py".

```
tms_ur_cv_odom_node = Node(
        name="tms_ur_cv_odom",
        package="tms_ur_construction",
        executable="<tms_ur_cv_odom or tms_ur_cv_posest, tms_ur_cv_odom_to_posest, tms_ur_cv_joint>",
        output="screen",
        remappings=[
            ("~/output/odom", "<your output topic name>"),
        ],
        parameters=[
            {
                "latest": LaunchConfiguration("latest"),
            },
            {
                "machine_name": "<your machine name>",
            },
        ],
    )
```

| executable file name | message type of read data | message type of output |
|--------|---------|---------|
|tms_ur_cv_odom | `nav_msgs::msg::Odometry` | `nav_msgs::msg::Odometry`|
|tms_ur_cv_posest | `geometry_msgs::msg::PoseStamped` | `geometry_msgs::msg::PoseStamped`|
|tms_ur_cv_odom_to_posest| `nav_msgs::msg::Odometry` |`geometry_msgs::msg::PoseStamped`|
|tms_ur_cv_joint | `sensor_msgs::msg::JointState` | `sensor_msgs::msg::JointState`|


```
# MongoDB manager
ros2 launch tms_db_manager tms_db_manager.launch.py

# Odometry and JointStats
ros2 launch tms_ur_construction tms_ur_cv_odom_demo_launch.py
```

#
#### Vizualizing waypoint by Rviz2 or OperaSimVR
If you want to visualize your waypoints in "Rviz2" or "OperaSimVR", please follow these steps:

 Rewrite waypoint parameter names in "tms_ur_waypoint_viz_launch.py" to its names you want to visualize.

```
self.waypoint_points = ["LOAD_POINT","R1_SIGNAL_POINT","RM1_R1_SIGNAL_POINT","R2_SIGNAL_POINT","R2_GOAL_POINT","RM2_R2_SIGNAL_POINT","R3_RELEASE_SIGNAL_POINT","RELEASE_POINT","RS1_R1_SIGNAL_POINT","RS1_R2_SIGNAL_POINT","RS2_R2_SIGNAL_POINT","RS2_R3_SIGNAL_POINT"]
```

```
# MongoDB manager
ros2 launch tms_db_manager tms_db_manager.launch.py

# Marker
ros2 launch tms_ur_construction tms_ur_waypoint_vizlaunch.py
```
#
#### Create control command parameters by OperaSimVR

When generating control command parameters such as position, orientation, and joint angles based on the posture of the construction machine model in "OperaSimVR", the model's position, orientation, and joint angles can be written to MongoDB and used as parameters for motion commands.


Run the following commands to store data in MongoDB and get the data.



#### Launch

Run the following commands to store parameters in MongoDB.

```
# MongoDB manager
ros2 launch tms_db_manager tms_db_manager.launch.py

# Odometry and JointStats
ros2 launch tms_ur_construction tms_ur_write_param_launch.py
```

The position and orientation data or joint core information is stored in "rostmsdb/parameter" on MongoDB.

For the configuration of "OperaSimVR", refer to the "OperaSimVR" README.
