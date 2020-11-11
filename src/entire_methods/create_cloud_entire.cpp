#pragma once
#include <iostream>
#include <vector>

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
                pcd.colors_.emplace_back(1.0, 1.0, 1.0);
            }
        }
    }
    return pcd;
}