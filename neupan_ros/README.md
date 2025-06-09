# NEUPAN ROS2 Wrapper

## System Requriement
- Ubuntu 22.04
- ROS2 Humble 
- nvidia driver
- cuda

## Set Neupan module
- clone the neupan repository

 ```
$ cd ~/
$ git clone https://github.com/hanruihua/NeuPAN
$ cd NeuPAN
```

- make virual enviroment and install the requirement
 ```
$ python -m venv neupan_venv
$ source ./neupan_venv/bin/activate
$ pip install -e . 
$ pip install ir-sim 
```

- And also set please add the virtual enviroment to get ros2 module
 ```
$ nano ./neupan_venv/bin/activate
```
- add this command to add ros2 module below the file

```
export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/lib/python3.10/site-packages
```

## Set neupan ros2 wrapper
- make workspace and clone
```
$ mkdir -p ~/neupan_ws/src
$ cd ~/nepan_ws/src
$ git clone https://github.com/Dangmu1996/neupan_ros.git

```

please change the python interpreter on code
```
/path/to/your/workspace/src/neupqn_ros/neupan_ros/nuepan_node.py
3rd line
venv_python = "/home/wego/NeuPAN/neupan_venv/bin/python" -> "/path/to/your/virual/enviroment/python/interpreter"
```

- build it and launch it
```
$ cd ~/neupan_ws
$ colcon build
$ source ~/NeuPAN/neupan_venv/bin/activate
$ source install/setup.bash
$ ros2 launch neupan_ros nupan_launch.py
```