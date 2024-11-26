# ROS-Robot-Control

## Building
```
git clone https://github.com/Erzis900/ROS-Robot-Control.git
cd ROS-Robot-Control
./init.sh
```

## Running
```
source install/setup.bash
ros2 run usb_cam usb_cam_node_exe
```

In separate terminal:
```
ros2 run camera_subscriber camera_node
```