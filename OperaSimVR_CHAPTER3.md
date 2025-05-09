### 3. Store and get data simultaneously in real-time

Run the following commands to store data in MongoDB and get the data.

Static terrain data is not included because it does not need to be acquired in real-time.

#### Launch

```
# MongoDB manager
ros2 launch tms_db_manager tms_db_manager.launch.py

# Odometry and JointStats
ros2 launch tms_sp_machine tms_sp_machine_odom_demo_launch.py

# Static terrain
python save_image_to_mongodb.py <inputFile>.las --output <outputImage>.png

```


#### Launch tms_ur_construction

Run the following commands to get data from MongoDB.

```
# Get odometry and jointstates
ros2 launch tms_ur_construction tms_ur_cv_odom_demo_launch.py

# Static terrain data
ros2 launch tms_ur_construction tms_ur_construction_terrain_mesh_launch.py filename_mesh:=<filename>

```

#### Play rosbag

```
ros2 bag play -l ./src/ros2_tms_for_construction/demo/demo2/rosbag2_2
```

After the end of rosbag, please check whether the data is stored to fs.chunks, fs.files, machine and sensor collection in your MongoDB.

GUI tool of MongoDB like a MongoDB Compass is easy to check them.

Here is an example. It may be a little different than yours, but as long as it is roughly the same, you should be fine.

![](demo/demo2/demo_mongodb_compass.png)


## Playmode (visualizeation finction)
The system receives the position, orientation, and joint angle data published by the real heavy machinery as ROS 2 topics, and updates the corresponding position, orientation, and joint angles of the machinery model within the system. As a result, the heavy machinery model in OperaSimVR moves in sync with the real-world machine.


**crawler dump**
| Type of information | Message type                       | Description                                                 |
| ------------------------------------- | ----------------------------------- | ----------------------------------------------------------- |
| position and orientation | `sensor_msgs::msg::JointState` `nav_msgs::msg::Odometry`      | Position and orientation of each machine. Plane Cartesian Coordinate System Reference. |
| angle of joints | `sensor_msgs::msg::JointState` | vessel angle of each machine |

**backhoe**
| Type of information | Message type                       | Description                                                 |
| ------------------------------------- | ----------------------------------- | ----------------------------------------------------------- |
| position and orientation | `sensor_msgs::msg::JointState` `nav_msgs::msg::Odometry`      | position and orientation of each machine. Plane Cartesian Coordinate System Reference .|
| angle of vessel | `sensor_msgs::msg::JointState` | angle of swing and boom, arm, bucket of each machine |

### about settings of subscriber 
Set from the "PoseSubscriber" attached to the construction machine.

**explanation of parameter**
| parameter name | description |
|--------|---------|
|ViaDB | Check this box if you want to retrieve information via a database. 
|WorldToMap | Check this box to convert the acquired coordinate values ​​from the world coordinate system to the map coordinate system.
|PoseMsgType | Select the topic type of the topic you want to subscribe to.
|SimPhysXSubscribeTopicName | Specify the topic name of PhysX location information (for operation test).
|SimAGXSubscribeTopicName | Specify the topic name of AGX location information (for operation test).
|RealSubscribeTopicName | Specify the topic name that publishes the position and joint information of the actual machine.
|ViaDBSubscribeTopicName | Specifies the topic name when going through a database.
|ChengePosition_sw | Check if you want to change the position of the construction machine model.
|MapMachinePosition | Don't change.
|MapMachineRotation | Don't change.


![](docs/OperaSimVR/PoseSubscriber.png)


### How to convert from world coordinate system to map coordinate system

The origin of the map coordinates in the world coordinate system is specified by the "Model_name" attached to each construction equipment object. Move the object's MapReferencePoint to the same position in the cyberspace field as the origin of the map coordinate system of the actual field.

**explanation of parameter**
| parameter name | description |
|--------|---------|
|Offset_x,Offset_y,Offset_z | Coordinates in the world coordinate system of the origin of the map coordinate system. y coordinate in x-plane Cartesian coordinates, z coordinate in y-plane Cartesian coordinates, x coordinate in z-plane Cartesian coordinates |
|OffsetRotation_x,y,z|  |



![](docs/OperaSimVR/ModelName.png)

