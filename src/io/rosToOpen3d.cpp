#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Open3D/Open3D.h>

void rosToOpen3d(const sensor_msgs::PointCloud2 &ros_pc2, open3d::geometry::PointCloud &o3d_pc, bool skip_colors = true)
{
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(ros_pc2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(ros_pc2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(ros_pc2, "z");
    o3d_pc.points_.reserve(ros_pc2.height * ros_pc2.width);
    if (ros_pc2.fields.size() == 3 || skip_colors == true)
    {
        for (size_t i = 0; i < ros_pc2.height * ros_pc2.width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
        {
            o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        }
    }
    else
    {
        o3d_pc.colors_.reserve(ros_pc2.height * ros_pc2.width);
        if (ros_pc2.fields[3].name == "rgb")
        {
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(ros_pc2, "r");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(ros_pc2, "g");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(ros_pc2, "b");

            for (size_t i = 0; i < ros_pc2.height * ros_pc2.width;
                 ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
            {
                o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
                o3d_pc.colors_.push_back(Eigen::Vector3d(((int)(*ros_pc2_r)) / 255.0, ((int)(*ros_pc2_g)) / 255.0,
                                                         ((int)(*ros_pc2_b)) / 255.0));
            }
        }
        else if (ros_pc2.fields[3].name == "intensity")
        {
            sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(ros_pc2, "intensity");
            for (size_t i = 0; i < ros_pc2.height * ros_pc2.width;
                 ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
            {
                o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
                o3d_pc.colors_.push_back(Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
            }
        }
    }
}