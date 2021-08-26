# ROS-noetic-packages
This repository is a detailed guide on creating a ROS Package using catkin
This README.md contains:
- Creating a catkin package
- Compiling the catkin workspace

## Creating a catkin package
This tutorial will demonstrate how to use catkin_create_pkg to create a new package.

Upon opening the terminal, change to the source directory of the catkin workspace:
```
cd catkin_ws/
cd src/
```

catkin_create_pkg requires you to give a a package name and optionally a few dependencies while creating the package:
```
# catkin_create_pkg package_name depend1 depend2 depend3
```
Now use catkin_create_pkg to create a new package called "ros_basics" which depends rospy and std_msgs:
```
catkin_create_pkg ros_basics rospy std_msgs
```
This will create a folder named ros_basics which contains an src folder, a CMakeLists.txt file and a package.xml file.

There are several other dependenices, like roscpp, that can be used based on your requirements while creating a package using catkin_create_pkg script.

## Compiling the catkin workspace
Now you need to build and compile the catkin workspace:
```
cd ..
catkin_make
```
