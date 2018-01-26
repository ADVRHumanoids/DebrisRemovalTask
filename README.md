First make sure you set:

```cpp
set_xbot_config $ROBOTOLOGY_ROOT/configs/ADVR_shared/user_example/walkman_example.yaml
```

Then to launch the module execute in different terminals:

```cpp
roslaunch walkman_gazebo walkman_debris_removal_task.launch
```

```cpp
NRTDeployer
```

```cpp
NRTDeployer $ROBOTOLOGY_ROOT/configs/ADVR_shared/user_example/walkman_debris.yaml

```

```cpp
XBotGUI $ROBOTOLOGY_ROOT/configs/ADVR_shared/user_example/walkman_debris_gui.yaml
```

To retrieve the pose of the end effectors from the terminal run:
```cpp
rosrun robot_state_publisher robot_state_publisher
```

```cpp
rosrun tf tf_echo /world_odom /RSoftHand
```