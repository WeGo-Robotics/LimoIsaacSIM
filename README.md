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

## JoyStick Application

You can control Limo with Joystick

The joystick which you can use is Xbox Joystick

![joystick](images/joystick.png)

Also you can connect your joystick with cable 

![joystick connection](images/joystick_connection.png)