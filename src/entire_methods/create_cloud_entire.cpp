#pragma once
#include <iostream>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Open3D/Open3D.h>

#include "../models/lidarParams.cpp"

using namespace std;
using namespace open3d;

geometry::PointCloud create_cloud_entire(vector<vector<double>> &grid, LidarParams &lidarParams, vector<vector<Eigen::Vector3d>> &color_grid)
{
    geometry::PointCloud pcd;
    for (int i = 0; i < lidarParams.height; i++)
    {
        for (int j = 0; j < lidarParams.width; j++)
        {
            if (grid[i][j] == 0)
            {
                continue;
            }

            double horizon_angle = lidarParams.horizon_res * j;
            double x = grid[i][j] * cos(horizon_angle * M_PI / 180);
            double y = grid[i][j] * sin(horizon_angle * M_PI / 180);

            double vertical_angle = lidarParams.vertical_res * i - lidarParams.bottom_angle;
            double z = grid[i][j] * tan(vertical_angle * M_PI / 180);

            pcd.points_.emplace_back(x, y, z);
            if (color_grid[i][j][0] > 0 || color_grid[i][j][1] > 0 || color_grid[i][j][2] > 0)
            {
                pcd.colors_.emplace_back(color_grid[i][j]);
            }
            else
            {
                pcd.colors_.emplace_back(0.1, 0.1, 0.1);
            }
        }
    }
    return pcd;
}

void create_cloud_entire2(vector<vector<double>> &src, LidarParams &lidarParams, sensor_msgs::PointCloud2 &dst)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < lidarParams.height; i++)
    {
        for (int j = 0; j < lidarParams.width; j++)
        {
            if (src[i][j] == 0)
            {
                continue;
            }

            double horizon_angle = lidarParams.horizon_res * j;
            double x = src[i][j] * cos(horizon_angle * M_PI / 180);
            double y = src[i][j] * sin(horizon_angle * M_PI / 180);

            double vertical_angle = lidarParams.vertical_res * i - lidarParams.bottom_angle;
            double z = src[i][j] * tan(vertical_angle * M_PI / 180);

            result->push_back(pcl::PointXYZ(x, y, z));
        }
    }

    pcl::toROSMsg(*result, dst);
    dst.header.frame_id = "os1_lidar";
}