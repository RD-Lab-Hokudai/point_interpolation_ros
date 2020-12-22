#pragma once

using namespace std;

struct LidarParams
{
    int height = 64;
    int width = 1024;
    double bottom_angle = 16.6 + 0.1;
    double horizon_res = 360.0 / width;
    double horizon_angle_min = 0;
    double horizon_angle_max = 0;
    double vertical_res = 33.2 / (height - 1);
};

LidarParams getDefaultLidarParams()
{
    LidarParams params;
    params.height = 64;
    params.width = 1024;
    params.bottom_angle = 16.6 + 0.1;

    params.horizon_res = 360.0 / params.width;
    params.vertical_res = 33.2 / (params.height - 1);

    return params;
};