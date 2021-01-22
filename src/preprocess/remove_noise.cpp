#pragma once
#include <vector>

#include <Open3D/Open3D.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
using namespace open3d;

void remove_noise(vector<vector<double> > &src, vector<vector<double> > &dst, LidarParams &lidarParams, double rad_coef = 0.001, int min_k = 1)
{
    int min_range = 3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr near(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr far(new pcl::PointCloud<pcl::PointXYZ>);
    int cnt = 0;
    for (int i = 0; i < src.size(); i++)
    {
        for (int j = 0; j < src[0].size(); j++)
        {
            if (src[i][j] <= 0)
            {
                continue;
            }

            double horizon_angle = lidarParams.horizon_res * j;
            double x = src[i][j] * cos(horizon_angle * M_PI / 180);
            double y = src[i][j] * sin(horizon_angle * M_PI / 180);

            double vertical_angle = lidarParams.vertical_res * i - lidarParams.bottom_angle;
            double z = src[i][j] * tan(vertical_angle * M_PI / 180);

            double distance2 = x * x + y * y + z * z;
            if (distance2 <= min_range * min_range)
            {
                near->push_back(pcl::PointXYZ(x, y, z));
                cnt++;
            }
            else
            {
                far->push_back(pcl::PointXYZ(x, y, z));
            }
        }
    }
    cout << cnt << endl;
    cout << near->size() << endl;

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(near);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int i = 0; i < near->size(); i++)
    {
        double x = near->points[i].x;
        double y = near->points[i].y;
        double z = near->points[i].z;
        double distance2 = x * x + y * y + z * z;

        //探索半径：係数*(距離)^2
        double radius = rad_coef * distance2;

        vector<int> pointIdxNKNSearch(min_k);
        vector<float> pointNKNSquaredDistance(min_k);

        int result = kdtree->radiusSearch(near->points[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance, min_k);
        if (result == min_k)
        {
            inliers->indices.push_back(i);
        }
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(near);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*near);

    *near += *far;
    for (int i = 0; i < near->size(); i++)
    {
        double x = near->points[i].x;
        double y = near->points[i].y;
        double z = near->points[i].z;

        double len = sqrt(x * x + y * y);

        double vertical_angle = atan2(z, len) * 180 / M_PI;
        int rowIdx = (vertical_angle + lidarParams.bottom_angle) / lidarParams.vertical_res;
        double horizon_angle = atan2(y, x) * 180 / M_PI;
        int columnIdx = horizon_angle / lidarParams.horizon_res;
        if (0 <= rowIdx && rowIdx < dst.size() && 0 <= columnIdx && columnIdx < dst[0].size())
        {
            dst[rowIdx][columnIdx] = len;
        }
    }
}

void remove_noise_cv(cv::Mat &src, cv::Mat &dst, cv::Mat &target_vs_mat, EnvParams envParams, double rad_coef = 0.001, int min_k = 1)
{
    auto ptr = make_shared<geometry::PointCloud>();
    src.forEach<double>([&ptr, &envParams, &target_vs_mat](double &now, const int position[]) -> void {
        if (now <= 0)
        {
            return;
        }

        double x = now * (position[1] - envParams.width / 2) / envParams.f_xy;
        double y = now * (target_vs_mat.at<int>(position[0], position[1]) - envParams.height / 2) / envParams.f_xy;
        ptr->points_.emplace_back(x, y, now);
    });
    auto kdtree = make_shared<geometry::KDTreeFlann>(*ptr);
    cv::Mat full_grid = cv::Mat::zeros(envParams.height, envParams.width, CV_64FC1);
    for (int i = 0; i < ptr->points_.size(); i++)
    {
        double x = ptr->points_[i][0];
        double y = ptr->points_[i][1];
        double z = ptr->points_[i][2];
        double distance2 = x * x + y * y + z * z;

        //探索半径：係数*(距離)^2
        double radius = rad_coef * distance2;

        //最も近い点を探索し，半径r以内にあるか判定
        vector<int> indexes;
        vector<double> dists;
        kdtree->SearchKNN(ptr->points_[i], min_k + 1, indexes, dists);

        //radiusを超えない範囲に近傍点があれば残す
        if (dists[min_k] <= radius)
        {
            int u = x / z * envParams.f_xy + envParams.width / 2;
            int v = y / z * envParams.f_xy + envParams.height / 2;
            full_grid.at<double>(v, u) = z;
        }
    }

    dst = cv::Mat::zeros(target_vs_mat.rows, target_vs_mat.cols, CV_64FC1);
    dst.forEach<double>([&](double &now, const int position[]) -> void {
        now = full_grid.at<double>(target_vs_mat.at<int>(position[0], position[1]), position[1]);
    });
}

void remove_noise_2d(vector<vector<double> > &src, vector<vector<double> > &dst, LidarParams &lidarParams, double rad_coef = 0.001, int min_k = 1)
{
    int rows = src.size();
    int cols = src[0].size();
    dst = vector<vector<double> >(rows, vector<double>(cols, 0));
    vector<vector<Eigen::Vector3d> > poses(rows, vector<Eigen::Vector3d>(cols));
    int r = 3;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (src[i][j] <= 0)
            {
                continue;
            }

            double horizon_angle = lidarParams.horizon_res * j;
            double x = src[i][j] * cos(horizon_angle * M_PI / 180);
            double y = src[i][j] * sin(horizon_angle * M_PI / 180);

            double vertical_angle = lidarParams.vertical_res * i - lidarParams.bottom_angle;
            double z = src[i][j] * tan(vertical_angle * M_PI / 180);
            poses[i][j] = Eigen::Vector3d(x, y, z);
        }
    }

    vector<vector<vector<int> > > neighbor_indices(rows, vector<vector<int> >(cols, vector<int>(8, -1)));
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    int dy[8] = {0, 1, 1, 1, 0, -1 - 1 - 1};
    for (int i = 0; i < 8; i++)
    {
    }

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (src[i][j] <= 0)
            {
                continue;
            }

            double horizon_angle = lidarParams.horizon_res * j;
            double distance2 = poses[i][j].norm();
            cout << poses[i][j] << " " << distance2 << endl;

            //探索半径：係数*(距離)^2
            double radius = rad_coef * distance2;
            int neighbors = 0;

            for (int ii = 0; ii < r; ii++)
            {
                for (int jj = 0; jj < r; jj++)
                {
                    int dx = jj - r / 2;
                    int dy = ii - r / 2;
                    if (dx == 0 && dy == 0)
                    {
                        continue;
                    }

                    int toY = i + dy;
                    int toX = j + dx;
                    if (toY < 0 || toY >= rows || toX < 0 || toX >= cols || src[toY][toX] <= 0)
                    {
                        continue;
                    }

                    double dist2 = (poses[i][j] - poses[toY][toX]).norm();

                    if (dist2 <= radius)
                    {
                        neighbors++;
                    }
                }
            }

            if (neighbors >= min_k)
            {
                dst[i][j] = src[i][j];
            }
        }
    }
}