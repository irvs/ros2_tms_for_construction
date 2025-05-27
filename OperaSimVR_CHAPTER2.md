

## §2. Control mode (controll finction)
In this system, real construction machinery can be remotely operated. By using a VR controller or keyboard, control commands are sent to the machinery via ROS2 topics. Just like in Play mode, position and orientation data are received from the real machinery through ROS2 topics. This data is used to animate the model of the machinery in the virtual environment, allowing the operator to monitor its movements in real time while performing remote operation.


### §2.1 When using in normal mode(Crawler dump)
0. Specify the topic name, speed, acceleration/deceleration, and the topic publishing interval before play. Start playback after completing the settings.
1. Switch to normal mode. 
2. Get into the model of the construction machine you want to operate. 
3. Turn on the controller. When using a VR headset, press the X button (Button.Three) on the VR controller. If not using VR, press the X key on the keyboard. Alternatively, you can set "OnOffSw" to "On" in the "VRCrawlerOp" script attached to each construction machine from the Inspector. 
4. Uncheck "UseRos2Topic" in the "VRCrawlerOp" script (unchecking this will prevent the machine from receiving topics from external sources). If operating with the keyboard, check the "Key" checkbox. 

From this point on, the construction machine you are in will respond to inputs from either the VR controller or the keyboard.

5. To end the operation, press the B button to dismount. (The controller (VRCrawlerOp) will be turned off.)

6. If you want to control it using an external topic, uncheck "UseRos2Topic" in "VRCrawlerOp".



### §2.2 When using in play mode(Crawler dump)
0. Specify the topic name, speed, acceleration/deceleration, and the topic publishing interval before play. Start playback after completing the settings.
1. Switch to normal mode. 
2. Get into the model of the construction machine you want to operate. 
3. Turn on the controller. When using a VR headset, press the X button (Button.Three) on the VR controller. If not using VR, press the X key on the keyboard. Alternatively, you can set "OnOffSw" to "On" in the "VRCrawlerOp" script attached to each construction machine from the Inspector. 

Note: From this point on, operating the VR controller or the keyboard will send command topics. Double-check in the Inspector that the values for movement commands such as forward motion and turning, as well as acceleration, are set appropriately.

4. To perform an emergency stop during operation, press the Y button on the left VR controller when using VR, or press the C key when using a keyboard. If a collision with the "geofence" occurs, the "emergency_sw" will also be set to true, triggering an emergency stop.
5. To end the operation, press the B button to dismount. (The controller (VRCrawlerOp) will be turned off.)


<img src=docs/OperaSimVR/VRCrawlerOp.png width="500px">

**explanation of parameter**
| parameter name | Units and message types | description |
|--------|---------|---------|
|OnOffSw | - |  Toggle the control function On or Off. When set to Off, the construction equipment model and the actual machine will not operate, even if the user is onboard and using VR controllers or a keyboard. However, the emergency stop function remains active even when set to Off.
|Emergency | - | Setting this to true will trigger an emergency stop of the actual machine. It sends a command to the emergency stop topic with motion instructions set to 0 [m/s] or 0 [rad/s]. Make sure to properly configure both the "EmergencyTopicName" and "RealPublishTopicName" before operating the actual machine.
|Key | - | Set to "false" when using a VR headset. Set to "true" when not using one.
|Use Ros2 Topic| - | Set to "false" if you want to operate the construction equipment model in "normal mode." Set to "true" if you want it to operate based on external commands without manual control within the system.
| SimPhysX Publish TopicName | geometry_msgs/msg/Twist [m/s][rad/s] | Set this option if you want to connect to PhysX and verify the operation (for testing purposes).
| SimAGX Publish TopicName | Geometry_msg/msg/Twist [m/s][rad/s] | Set this option if you want to connect to AGX and verify the operation (for testing purposes).
|Real Publish TopicName| Geometry_msg/msg/Twist [m/s][rad/s] | Set the topic name for sending motion commands to the actual machine.
|Emergncy TopicName| std_msgs/msg/Bool | The topic name for sending emergency stop commands.
|LinearSpeed| [m/s] | Specifies the maximum value [m/s] for forward and backward (linear) motion commands sent to the actual machine. When the VR D-pad (Axis2D.SecondaryThumbstick) is fully tilted or the arrow keys are pressed, the machine will accelerate up to this speed.
|RotSpeed| [rad/s] | Specifies the maximum value [rad/s] for rotational (angular) motion commands sent to the actual machine. When the VR D-pad (Axis2D.SecondaryThumbstick) is fully tilted or the arrow keys are pressed, the machine will accelerate up to this turning speed.
|Max Linear Acceleration| [m/s^2] | Specifies the maximum acceleration value used when accelerating toward the commanded forward or backward (linear) motion.
|Max Linear Deceleration| [m/s^2] |Specifies the maximum deceleration value used when slowing down to the commanded forward or backward (linear) motion.
|Max Angular Acceleration| [rad/s^2] |Specifies the maximum angular acceleration value used when accelerating to the commanded rotational speed.
|Max Angular Deceleration| [rad/s^2] |Specifies the maximum angular deceleration value used when slowing down to the commanded rotational speed.
|Publish Message Interval| [s]|The interval [s] at which the command topic is published.

The remaining parameters do not need to be set when using the control mode (they are used in preview mode).


### §2.3 Operation Method(Crawler dump)
**When you use VR goggles**
| operation | controller operation |
|--------|---------|
|When the controller is not turned on | Press the X button (Button.Three) to turn it on.
|Forward and backward (linear) and rotation (angular) | Use the D-pad (Axis2D.PrimaryThumbstick) on the left controller to move. Pushing it forward moves machine forward, backward moves machine back, left rotate left, and right rotate right.
|Emergency stop | Press the Y button (Button.Four).
|Release emergency stop | While holding down the right controller's trigger (Axis1D.SecondaryIndexTrigger), press the Y button (Button.Four).
|Turn off the controller. | press the B button (Button.Two) on the right controller.


----


**When you  operate from the keyboard**
| operation | controller operation |
|--------|---------|
|When the controller is not turned on | Press the X key on your keyboard to turn it on.
|Forward and backward (linear) and rotation (angular) | Use the arrow keys on the keyboard to move. Press the up arrow to move forward, the down arrow to move backward, the left arrow rotate left, and the right arrow rotate right.
|Emergency stop | Press the C key.
|Release emergency stop | Shift key + C key.
|Turn off the controller. | press the B key.


## §3. Preview mode (preview finction)
For the crawler dump, there is a feature that allows remote operation of the real machine while simultaneously simulating the operation of its model in cyberspace. Currently, there is a significant discrepancy in the simulation, so improvements are needed, but it is functional during straight-line movement. The simulation operates while correcting the discrepancies between the simulation and the real machine.

#### usage
Set "VRCrawleOp" before "Play" (for details, refer to the section on remote operation. Parameters used only in "Preview mode" are explained below).
1. Switch to "Play mode" once to align the position of the construction machine model with the real machine.
2. Switch the mode to "PreviewMode".
3. Board the crawler dump you want to operate. Turn on the controller "VRCrawleOp".
4. Operate in the same way as remote control during "Play mode".
The construction machine model in cyberspace will simulate the operation as it moves. The real machine operates with a delay corresponding to the "Time_Delay" from this simulated operation. Once a straight movement command is sent, only straight movement commands can be sent. Therefore, to switch between turning and straight movement commands, change the value of "LinearOrRot."
5. If the discrepancy becomes too large, switch to "Play mode" to correct the position.


**explanation of parameter**
| parameter name | Units and message types | description |
|--------|---------|---------|
|Time_Deray | s |  The time delay between sending an operation command to the construction machine in cyberspace and sending the operation command to the real machine.
|intervalInMilisecond|ms|Interval for applying position correction.|
| LinearOrRot| - | Switch between straight and turning movements. Use 1 for straight and 2 for turning. When set to 0, the mode automatically switches upon sending an operation command. |
