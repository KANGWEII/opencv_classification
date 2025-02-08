# Running Guide
This guide will walk you through running the sample classification task.

## Camera Bringup
> **Remember to source the ROS Workspace first**

> **Refer to the [usb_cam guide](https://github.com/ros-drivers/usb_cam) to configure your camera**

```bash
$ ros2 launch usb_camera camera.launch.py
```

## Classification Bringup
> **Remember to source the ROS Workspace first**

```bash
$ ros2 run opencv_classification visual_processing
```