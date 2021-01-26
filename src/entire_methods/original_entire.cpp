#pragma once
#include <iostream>
#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "../models/envParams.cpp"
#include "../models/hyperParams.cpp"
#include "../models/lidarParams.cpp"

#include "../utils/SegmentationGraph.cpp"
#include "../utils/UnionFind.cpp"

using namespace std;

void original_entire(vector<vector<double>> &grid, EnvParams &envParams, HyperParams &hyperParams, LidarParams &lidarParams, cv::Mat &img, int horizon_offset, vector<vector<Eigen::Vector3d>> &color_grid)
{
    auto start = chrono::system_clock::now();
    vector<vector<double>> noise_removed;
    //remove_noise_2d(grid, noise_removed, lidarParams, 0.01, 1);
    //grid = noise_removed;

    //remove_noise(grid, noise_removed, lidarParams);

    vector<vector<vector<int>>> image_positions(lidarParams.height, vector<vector<int>>(lidarParams.width, vector<int>(2, 0)));
    {
        // Calibration
        double rollVal = (envParams.roll - 500) / 1000.0;
        double pitchVal = (envParams.pitch - 500) / 1000.0;
        double yawVal = (envParams.yaw - 500) / 1000.0;
        double dx = (envParams.X - 500) / 100.0;
        double dy = (envParams.Y - 500) / 100.0;
        double dz = (envParams.Z - 500) / 100.0;

        Eigen::MatrixXd calibration_mtx(3, 4);
        calibration_mtx << cos(yawVal) * cos(pitchVal), cos(yawVal) * sin(pitchVal) * sin(rollVal) - sin(yawVal) * cos(rollVal), cos(yawVal) * sin(pitchVal) * cos(rollVal) + sin(yawVal) * sin(rollVal), dx,
            sin(yawVal) * cos(pitchVal), sin(yawVal) * sin(pitchVal) * sin(rollVal) + cos(yawVal) * cos(rollVal), sin(yawVal) * sin(pitchVal) * cos(rollVal) - cos(yawVal) * sin(rollVal), dy,
            -sin(pitchVal), cos(pitchVal) * sin(rollVal), cos(pitchVal) * cos(rollVal), dz;
        for (int i = 0; i < lidarParams.height; i++)
        {
            for (int j = 0; j < lidarParams.width; j++)
            {
                int j2 = (j - horizon_offset + lidarParams.width) % lidarParams.width;
                double horizon_angle = lidarParams.horizon_res * j2;
                if (horizon_angle <= 90 || horizon_angle >= 270)
                {
                    continue;
                }

                double vertical_angle = lidarParams.vertical_res * i - lidarParams.bottom_angle;
                double r = 10; // rを仮定しても計算に必要な角度情報は変わらない
                double rawX = r * sin(horizon_angle * M_PI / 180);
                double rawY = -r * tan(vertical_angle * M_PI / 180);
                double rawZ = -r * cos(horizon_angle * M_PI / 180);

                double x = calibration_mtx(0, 0) * rawX + calibration_mtx(0, 1) * rawY + calibration_mtx(0, 2) * rawZ + calibration_mtx(0, 3);
                double y = calibration_mtx(1, 0) * rawX + calibration_mtx(1, 1) * rawY + calibration_mtx(1, 2) * rawZ + calibration_mtx(1, 3);
                double z = calibration_mtx(2, 0) * rawX + calibration_mtx(2, 1) * rawY + calibration_mtx(2, 2) * rawZ + calibration_mtx(2, 3);

                int u = (int)(envParams.width / 2 + envParams.f_xy * x / z);
                int v = (int)(envParams.height / 2 + envParams.f_xy * y / z);
                if (0 <= u && u < envParams.width && 0 <= v && v < envParams.height)
                {
                    image_positions[i][j][0] = v;
                    image_positions[i][j][1] = u;
                    cv::Vec3b c = img.at<cv::Vec3b>(v, u);
                    color_grid[i][j] = Eigen::Vector3d(c[0] / 255.0, c[1] / 255.0, c[2] / 255.0);
                }
            }
        }
    }

    double time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - start).count();
    cout << "positioning " << time << endl;

    shared_ptr<UnionFind> color_segments;
    {
        // Segmentation
        SegmentationGraph graph(&img);
        time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - start).count();
        cout << "generation " << time << endl;
        color_segments = graph.segmentate(hyperParams.original_color_segment_k);
    }

    time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - start).count();
    cout << "segmentation " << time << endl;

    vector<vector<double>> interpolated_grid(lidarParams.height, vector<double>(lidarParams.width, 0));
    {
        // Enhancement
        for (int i = 0; i < lidarParams.height; i++)
        {
            for (int j = 0; j < lidarParams.width; j++)
            {
                if (grid[i][j] > 0)
                {
                    // すでに存在する点はそのまま入れる
                    interpolated_grid[i][j] = grid[i][j];
                    continue;
                }

                int img_u = image_positions[i][j][1];
                int img_v = image_positions[i][j][0];
                if (img_u == 0 && img_v == 0)
                {
                    continue;
                }
                double coef = 0;
                double val = 0;

                cv::Vec3b d0 = img.at<cv::Vec3b>(img_v, img_u);
                int r0 = color_segments->root(img_v * envParams.width + img_u);

                for (int ii = 0; ii < hyperParams.original_r; ii++)
                {
                    for (int jj = 0; jj < hyperParams.original_r; jj++)
                    {
                        int dy = ii - hyperParams.original_r / 2;
                        int dx = jj - hyperParams.original_r / 2;
                        if (i + dy < 0 || i + dy >= lidarParams.height)
                        {
                            continue;
                        }
                        if (grid[i + dy][(j + dx + lidarParams.width) % lidarParams.width] == 0)
                        {
                            continue;
                        }

                        int img_u_tmp = image_positions[i + dy][(j + dx + lidarParams.width) % lidarParams.width][1];
                        int img_v_tmp = image_positions[i + dy][(j + dx + lidarParams.width) % lidarParams.width][0];
                        if (img_u_tmp == 0 && img_v_tmp == 0)
                        {
                            continue;
                        }

                        cv::Vec3b d1 = img.at<cv::Vec3b>(img_v_tmp, img_u_tmp);
                        int r1 = color_segments->root(img_v_tmp * envParams.width + img_u_tmp);
                        double tmp = exp(-(dx * dx + dy * dy) / 2 / hyperParams.original_sigma_s / hyperParams.original_sigma_s); // * exp(-cv::norm(d0 - d1) / 2 / hyperParams.original_sigma_r / hyperParams.original_sigma_r);
                        if (r1 != r0)
                        {
                            tmp *= hyperParams.original_coef_s;
                            //tmp = 0;
                        }
                        coef += tmp;
                        val += tmp * grid[i + dy][(j + dx + lidarParams.width) % lidarParams.width];
                    }
                }
                if (coef > 0 /* some threshold */)
                {
                    interpolated_grid[i][j] = val / coef;
                }
            }
        }
    }

    //remove_noise(interpolated_grid, noise_removed, lidarParams, 0.001, 2);
    //interpolated_grid = noise_removed;

    grid = interpolated_grid;

    time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - start).count();
    cout << "enhancement " << time << endl;

    /*
    {
        // Apply
        for (int i = 0; i < lidarParams.height; i++)
        {
            for (int j = 0; j < lidarParams.width; j++)
            {
                if (grid[i][j])
                {
                    continue;
                }
                grid[i][j] = interpolated_grid[i][j];
            }
        }
    }
    */
}