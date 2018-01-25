**Valve Task**

**Set xbot config**
```
set_xbot_config /home/super/advr-superbuild/configs/ADVR_shared/user_example/walkman_valve_task.yaml

```


**Launch**
```
roslaunch walkman_gazebo walkman_valve_task.launch
```

```
NRTDeployer
```

```
XBotGUI
```


**GUI**
1) HomingExample->Start->Stop
2) DebrisRemovalTask -> Start
3) Pub valve pose(for right hand):
```
rostopic pub /valve_pose ADVR_ROS/im_pose_msg "name: ''
pose_stamped:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  pose:
    position:
      x: 0.8
      y: -0.4
      z: 1.4
    orientation:
      x: 0.0
      y: -0.7071068
      z: 0.0
      w: 0.7071068"
```
   
           
4) press on button _success_ or _fail_ to make finite state machine transits to next state.