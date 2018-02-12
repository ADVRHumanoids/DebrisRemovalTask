First make sure you set:

```cpp
set_xbot_config $ROBOTOLOGY_ROOT/configs/ADVR_shared/user_example/centauro_debris.yaml
```

Then to launch the module in simulation execute in different terminals:

```cpp
roslaunch centauro_gazebo centauro_debris_removal_task.launch
```

```cpp
NRTDeployer
```

```cpp
XBotGUI
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