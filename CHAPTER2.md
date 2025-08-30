### 3. Make motion parameters from "OperaSimVR"
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
