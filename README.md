# URL
- The wiki page you can refer here

https://goofy-pleasure-a84.notion.site/Limo-ISAACSIM-Example-Package-1ff09b2ac4bf806886e1e13976cf80f0?source=copy_link

## Compatible Version
- Ubuntu 22.04
- ROS2 Humble

## Basic Setup
- Setup for workspace and install dependency package.

```
$ cd ~/
$ mkdir -p wego_ws/src
$ cd ~/wego_ws/src
$ git clone -b ros2 https://github.com/WeGo-Robotics/LimoIsaacSIM.git
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build
```
## View Sensor data 
You can see the sensor data with RViz
```
# With Limo Isaac sim played
$ cd ~/wego_ws
$ source install/setup.bash
$ ros2 launch wego display_launch.py
```
![display](images/display.png)


## JoyStick Application

You can control Limo with Joystick

The joystick which you can use is Xbox Joystick

![joystick](images/joystick.png)

Also you can connect your joystick with cable 

![joystick connection](images/joystick_connection.png)

After Connecting the Joystick you can launch the application

```
# With Limo Isaac sim played
$ cd ~/wego_ws
$ source install/setup.bash
$ ros2 launch wego joystick_launch.py
```

You can control limo with joystick shown the picture below
![joystick control](images/joystick_control.png)

The Joystick Application is designed with a structure that allows for easy transitions between autonomous and manual control modes.
![joystick system](images/joystick_system.png)

## Scan filter

You can filter the lidar data which detect Limo's body itself,

 ```
# With Limo Isaac sim played
$ cd ~/wego_ws
$ source install/setup.bash
$ ros2 launch wego scan_filter_launch.py
```

This application will filter out the lidar scan data like this.
![lidar filter](images/lidar_filter.png)

## Navigation Application

You can test ROS2 Navigation With Limo Isaac Sim
 ```
# With Limo Isaac sim played
$ cd ~/wego_ws
$ source install/setup.bash

# For the first terminal(laser filter, joystick bridge)
$ ros2 launch wego preset_launch.py

# Second terminal launch navigation
$ ros2 launch wego navigation_diff_launch.py

# Third terminal execute navigation example code
$ ros2 run wego patrol_limo_navigation
```

Limo will patrol these points
![limo navigatin](images/navigation.png)


## E2E Navigation: NeuPAN Integration

This project includes a ROS2 + Isaac Sim integration of [NeuPAN](https://github.com/hanruihua/NeuPAN),  
an end-to-end neural planner originally developed by Han Ruihua.

This repository and its modifications are distributed under the terms of the  
**GNU General Public License v3 (GPLv3)**, in accordance with the original license.

For more details, see the [LICENSE](./LICENSE) file.

- 📄 NeuPAN Website: https://hanruihua.github.io/neupan_project/  
- 📦 NeuPAN GitHub: https://github.com/hanruihua/NeuPAN
