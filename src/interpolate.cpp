#pragma once
#include <vector>
#include <chrono>

#include <Open3D/Open3D.h>
#include <opencv2/opencv.hpp>
#include <time.h>

#include "models/envParams.cpp"
#include "models/hyperParams.cpp"
#include "data/loadParams.cpp"
#include "preprocess/grid_pcd.cpp"
#include "methods/linear.cpp"
#include "methods/mrf.cpp"
#include "methods/pwas.cpp"
#include "methods/original.cpp"
#include "postprocess/restore_pcd.cpp"

using namespace std;
using namespace open3d;

geometry::PointCloud interpolate(cv::Mat &img, geometry::PointCloud &pcd, EnvParams envParams, HyperParams hyperParams)
{
    cv::Mat blured;
    cv::GaussianBlur(img, blured, cv::Size(3, 3), 0.5);

    auto pcd_ptr = make_shared<geometry::PointCloud>();
    *pcd_ptr = pcd;

    vector<vector<double>>
        original_grid, filtered_grid, original_interpolate_grid, filtered_interpolate_grid;
    vector<vector<int>> target_vs, base_vs;
    int layer_cnt = 16;
    calc_grid(pcd_ptr, envParams, original_grid, filtered_grid, original_interpolate_grid, filtered_interpolate_grid, target_vs, base_vs, layer_cnt);

    vector<vector<int>> original_vs;
    if (envParams.isFullHeight)
    {
        original_vs = vector<vector<int>>(envParams.height, vector<int>(envParams.width, 0));
        for (int i = 0; i < envParams.height; i++)
        {
            for (int j = 0; j < envParams.width; j++)
            {
                original_vs[i][j] = i;
            }
        }
        swap(original_vs, target_vs);
    }
    else
    {
        original_vs = target_vs;
    }

    vector<vector<double>> interpolated_z;
    if (envParams.method == "linear")
    {
        linear(interpolated_z, filtered_interpolate_grid, target_vs, base_vs, envParams);
    }
    if (envParams.method == "mrf")
    {
        mrf(interpolated_z, filtered_interpolate_grid, filtered_interpolate_grid, target_vs, base_vs, envParams, blured,
            hyperParams.mrf_k, hyperParams.mrf_c);
    }
    if (envParams.method == "pwas")
    {
        pwas(interpolated_z, filtered_interpolate_grid, target_vs, base_vs, envParams, blured,
             hyperParams.pwas_sigma_c, hyperParams.pwas_sigma_s,
             hyperParams.pwas_sigma_r, hyperParams.pwas_r);
    }
    if (envParams.method == "original")
    {
        original(interpolated_z, filtered_interpolate_grid, target_vs, base_vs, envParams, blured,
                 hyperParams.original_color_segment_k, hyperParams.original_sigma_s,
                 hyperParams.original_sigma_r, hyperParams.original_r, hyperParams.original_coef_s);
    }

    cv::Mat grid_img = cv::Mat::zeros(target_vs.size(), envParams.width, CV_8UC3);
    auto filtered_ptr = make_shared<geometry::PointCloud>();
    {
        for (int i = 0; i < original_vs.size(); i++)
        {
            for (int j = 0; j < envParams.width; j++)
            {
                if (original_grid[i][j] > 0)
                {
                    grid_img.at<cv::Vec3b>(i, j) = cv::Vec3b(100 * filtered_grid[i / 4][j], 100 * filtered_grid[i / 4][j], 0);
                    double z = original_grid[i][j];
                    double x = z * (j - envParams.width / 2) / envParams.f_xy;
                    double y = z * (target_vs[i][j] - envParams.height / 2) / envParams.f_xy;
                    if (i % (64 / layer_cnt) == 0)
                    {
                        filtered_ptr->points_.emplace_back(x, y, z);
                    }
                }
            }
        }

        //cv::imshow("aa", grid_img);
        //cv::waitKey();
        //visualization::DrawGeometries({filtered_ptr});
    }

    auto interpolated_ptr = make_shared<geometry::PointCloud>();
    auto original_ptr = make_shared<geometry::PointCloud>();
    restore_pcd(interpolated_z, original_grid, target_vs, original_vs, envParams, blured, interpolated_ptr, original_ptr);

    return *interpolated_ptr;
}
