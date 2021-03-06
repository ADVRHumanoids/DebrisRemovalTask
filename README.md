First make sure you add to walkman_example.yaml the DebrisRemovalTask module and to set it:

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

To send a pose to be reached by the locomotion
```cpp
./advr-superbuild/external/chengxu_walking/python/walkTo.py -3.13 1.24 90
```
