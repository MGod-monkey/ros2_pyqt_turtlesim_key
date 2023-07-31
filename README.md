## A pyqt demo to control the movement of turtlesim in ros2

Environment:

- ROS2：foxy

- python：3.8.10
- PyQt：5.10.12

## 1. Initial packages and workspaces

```sh
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/MGod-monkey/ros2_pyqt_turtlesim_key.git
```

## 2. Build and Run

```sh
cd ~/dev_ws
colcon build
source install/setup.sh
ros2 run ros2_pyqt_turtlesim_key ros2_pyqt_turtlesim_key
```

## 3.Other

You can open the `ros2_pyqt_turtlesim_key/key.ui` file through Qt designer to edit the graphical interface, and convert the `.ui` file into a python file through the following command

```sh
pyuic5 -o key_ui.py key.ui
```

