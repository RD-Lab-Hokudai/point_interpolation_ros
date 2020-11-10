#pragma once
#include <iostream>
#include <vector>

#include <Open3D/Open3D.h>

#include "../models/lidarParams.cpp"

using namespace std;
using namespace open3d;

void linear_entire(vector<vector<double>> &grid, LidarParams &lidarParams)
{
    for (int i = 4; i < lidarParams.height; i += 4)
    {
        for (int j = 0; j < lidarParams.width; j++)
        {
            if (grid[i - 4][j] == 0 && grid[i][j] == 0)
            {
                continue;
            }

            double angle1 = lidarParams.vertical_res * (i - 4) - lidarParams.bottom_angle;
            double r1 = grid[i - 4][j];
            double z1 = grid[i - 4][j] * tan(angle1 * M_PI / 180);

            double angle2 = lidarParams.vertical_res * i - lidarParams.bottom_angle;
            double r2 = grid[i][j];
            double z2 = grid[i][j] * tan(angle2 * M_PI / 180);

            for (int k = i - 3; k < i; k++)
            {
                if (grid[k][j] > 0)
                {
                    continue;
                }

                double angle_tmp = lidarParams.vertical_res * k - lidarParams.bottom_angle;
                double tan_tmp = tan(angle_tmp * M_PI / 180);
                double divider = z1 - r1 * tan_tmp + r2 * tan_tmp - z2;
                if (divider != 0)
                {
                    double s = (r2 * tan_tmp - z2) / divider;
                    grid[k][j] = s * r1 + (1 - s) * r2;
                }
            }
        }
    }

    for (int j = 0; j < lidarParams.width; j++)
    {
        for (int i = lidarParams.height - 4 + 1; i < lidarParams.height; i++)
        {
            grid[i][j] = grid[lidarParams.height - 4][j];
        }
    }
}