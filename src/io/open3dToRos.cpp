#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Open3D/Open3D.h>

using namespace std;

void open3dToRos(const open3d::geometry::PointCloud &pointcloud, sensor_msgs::PointCloud2 &ros_pc2, std::string frame_id)
{
    sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
    if (pointcloud.HasColors())
    {
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    }
    else
    {
        modifier.setPointCloud2FieldsByString(1, "xyz");
    }
    modifier.resize(pointcloud.points_.size());
    ros_pc2.header.frame_id = frame_id;
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
    if (pointcloud.HasColors())
    {
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
        for (size_t i = 0; i < pointcloud.points_.size();
             i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
        {
            const Eigen::Vector3d &point = pointcloud.points_[i];
            const Eigen::Vector3d &color = pointcloud.colors_[i];
            *ros_pc2_x = point(0);
            *ros_pc2_y = point(1);
            *ros_pc2_z = point(2);
            *ros_pc2_r = (int)(255 * color(0));
            *ros_pc2_g = (int)(255 * color(1));
            *ros_pc2_b = (int)(255 * color(2));
        }
    }
    else
    {
        for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
        {
            const Eigen::Vector3d &point = pointcloud.points_[i];
            *ros_pc2_x = point(0);
            *ros_pc2_y = point(1);
            *ros_pc2_z = point(2);
        }
    }
}