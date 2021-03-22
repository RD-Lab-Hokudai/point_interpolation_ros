## Point cloud interpolater for ROS

A point interpolation library for LiDAR using images.

This is a ROS package version.

### Features

- Supports Linear and Original method.
- Interpolate sensor_msgs/PointCloud2 upto same resolution as sensor_msgs/Image

### Requirements

- ROS melodic
- C++ 17
- PCL 1.8
- OpenCV 3.4.2

### How to use

1. Set topic names

In 'launch/point_interpolation.launch',

```
...

<launch>
    ...

    <param name="thermal_node" type="string" value="<thermal image topic name>"/>
    <param name="rgb_node" type="string" value="<RGB image topic name>"/>
    <param name="points_node" type="string" value="<PointCloud topic name>"/>

    ...
</launch>
```

This package requires both thermal and RGB image topics.

2. Compile and run nodes

In your ROS workspace,

```
$ catkin_make
$ ros launch point_interpolation.launch
```

If you will save images and point clouds, check 'src/adapter_node.cpp'.
