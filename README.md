# omniBot
A omnirectional robot develop using ROS2

![](https://i.ibb.co/jWGmQsK/image.png)

## Install the packages

- First, create a ROS workspace and clone this repository inside yout /src folder:
```
$ mkdir -p ~/omnibot_ws/src
$ cd ~/omnibot_ws/src
$ git clone git@github.com:felipemohr/omniBot.git
```

- Install all dependencies and build your workspace:
```
$ cd ~/omnibot_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=foxy -y
$ colcon build
```

## Simulation

When open a new terminal:
```
$ cd ~/omnibot_ws
$ . install/setup.bash
```

Start the simulation:
`$ ros2 launch omnibot_ignition start_simulation.launch.py`

Use `use_rviz:=true` if you want to open RViz.

### Teleoperation

If you want to run a teleoperation with a joystick, run:
`$ ros2 launch omnibot_control joy_teleop.launch.py`

If you want to run a teleoperation with your keyboard, run:
`$ ros2 launch omnibot_control keyboard_teleop.launch.py`

### SLAM

Run EKF Localization:
`ros2 launch omnibot_navigation ekf_localization.launch.py`

Run SLAM:
`ros2 launch omnibot_navigation slam_online_async.launch.py`
