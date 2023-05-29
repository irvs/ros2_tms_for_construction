# tms_ur_gui

tms_ur_gui is a package for transforming construction data (ex. machine's location, terrain and hardness of ground) using GUI tool.

# Usecase

Edit [params_file](./params_file.yaml).

```
ground_tf_broadcaster:
  ros__parameters:
     from_frame: ground_frame
     to_frame: world
odom_tf_broadcaster:
  ros__parameters:
     from_frame: odom_frame
     to_frame: world
```

Run the following command.

```
ros2 run tms_tf_gui ground_tf_broadcaster --ros-args --params-file params_file.yaml

ros2 run tms_tf_gui odom_tf_broadcaster --ros-args --params-file params_file.yaml
```

<!-- TODO: Update this README.md  -->
