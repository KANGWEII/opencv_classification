# Installation Guide

## Getting this repository
```bash
$ git clone https://github.com/KANGWEII/opencv_classification.git
$ cd opencv_classification
```

## Install Dependencies
### ROS Driver Packages
All ROS driver packages are listed in the [classification.repos](/classification.repos) file. You can import the ROS driver packages by running the following command:
```bash
$ vcs import --recursive . < ./classification.repos
```

## Build the Workspace
After installing the dependencies, you can build the workspace by running the following commands:
```bash
$ source /opt/ros/humble/setup.bash
$ colcon build --symlink-install
```