#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <open3d_test/PointsImages.h>

#include <cv_bridge/cv_bridge.h>
#include <Open3D/Open3D.h>
#include <opencv2/opencv.hpp>

#include "io/open3dToRos.cpp"
#include "io/rosToOpen3d.cpp"

#include "models/envParams.cpp"
#include "models/hyperParams.cpp"
#include "models/lidarParams.cpp"
#include "data/loadParams.cpp"
#include "entire_methods/grid_entire.cpp"
#include "entire_methods/original_entire.cpp"
#include "entire_methods/linear_entire.cpp"
#include "entire_methods/create_cloud_entire.cpp"

using namespace std;
using namespace open3d;

EnvParams params_use;
HyperParams hyper_params;
LidarParams lidar_params;

ros::Publisher _pub;

/*
geometry::PointCloud interpolate(geometry::PointCloud &pcd, cv::Mat &rgb_front, cv::Mat &rgb_right, cv::Mat &rgb_back, cv::Mat &rgb_left)
{
    open3d::geometry::PointCloud pcd_front, pcd_right, pcd_back, pcd_left;
    for (int i = 0; i < pcd.points_.size(); i++)
    {
        double x = pcd.points_[i][0];
        double y = pcd.points_[i][1];
        double z = pcd.points_[i][2];

        if (x < 0)
        {
            pcd_front.points_.emplace_back(x, y, z);
        }
        if (y > 0)
        {
            pcd_right.points_.emplace_back(-y, x, z);
        }
        if (x > 0)
        {
            pcd_back.points_.emplace_back(-x, -y, z);
        }
        if (y < 0)
        {
            pcd_left.points_.emplace_back(y, -x, z);
        }
    }

    open3d::geometry::PointCloud res_front, res_right, res_back, res_left;

#pragma omp parallel
    {
#pragma omp sections
        {
#pragma omp section
            {
                res_front = interpolate(rgb_front, pcd_front, params_use, hyper_params);
            }
#pragma omp section
            {
                res_right = interpolate(rgb_right, pcd_right, params_use, hyper_params);
            }
#pragma omp section
            {
                res_back = interpolate(rgb_back, pcd_back, params_use, hyper_params);
            }
#pragma omp section
            {
                res_left = interpolate(rgb_left, pcd_left, params_use, hyper_params);
            }
        }
    }

    open3d::geometry::PointCloud res_pcd;

    for (int i = 0; i < res_front.points_.size(); i++)
    {
        res_pcd.points_.emplace_back(res_front.points_[i]);
    }
    for (int i = 0; i < res_right.points_.size(); i++)
    {
        double x = res_right.points_[i][0];
        double y = res_right.points_[i][1];
        double z = res_right.points_[i][2];
        res_pcd.points_.emplace_back(y, -x, z);
    }
    for (int i = 0; i < res_back.points_.size(); i++)
    {
        double x = res_back.points_[i][0];
        double y = res_back.points_[i][1];
        double z = res_back.points_[i][2];
        res_pcd.points_.emplace_back(-x, -y, z);
    }
    for (int i = 0; i < res_left.points_.size(); i++)
    {
        double x = res_left.points_[i][0];
        double y = res_left.points_[i][1];
        double z = res_left.points_[i][2];
        res_pcd.points_.emplace_back(-y, x, z);
    }

    return res_pcd;
}
*/

void interpolate_original4(vector<vector<double>> &grid, cv::Mat &rgb_front, cv::Mat &rgb_right, cv::Mat &rgb_back, cv::Mat &rgb_left, vector<vector<Eigen::Vector3d>> &color_grid)
{
#pragma omp parallel
    {
#pragma omp sections
        {
#pragma omp section
            {
                original_entire(grid, params_use, hyper_params, lidar_params, rgb_front, 0, color_grid);
            }
#pragma omp section
            {
                original_entire(grid, params_use, hyper_params, lidar_params, rgb_right, -lidar_params.width / 4, color_grid);
            }
#pragma omp section
            {
                original_entire(grid, params_use, hyper_params, lidar_params, rgb_back, -lidar_params.width / 2, color_grid);
            }
#pragma omp section
            {
                original_entire(grid, params_use, hyper_params, lidar_params, rgb_left, lidar_params.width / 4, color_grid);
            }
        }
    }
}

void onDataReceive(const open3d_test::PointsImages &data)
{
    cv::Mat rgb_front = cv_bridge::toCvCopy(data.rgb_front)->image;
    cv::Mat rgb_right = cv_bridge::toCvCopy(data.rgb_right)->image;
    cv::Mat rgb_back = cv_bridge::toCvCopy(data.rgb_back)->image;
    cv::Mat rgb_left = cv_bridge::toCvCopy(data.rgb_left)->image;

    open3d::geometry::PointCloud pcd;
    rosToOpen3d(data.points, pcd);

    //auto res_pcd = interpolate(pcd, rgb_front, rgb_right, rgb_back, rgb_left);
    auto grid = grid_entire(pcd, lidar_params);
    vector<vector<Eigen::Vector3d>> color_grid(lidar_params.height, vector<Eigen::Vector3d>(lidar_params.width));
    interpolate_original4(grid, rgb_front, rgb_right, rgb_back, rgb_left, color_grid);
    //original_entire(grid, params_use, hyper_params, lidar_params, rgb_right, -lidar_params.width / 4, color_grid);
    linear_entire(grid, lidar_params);
    auto res_pcd = create_cloud_entire(grid, lidar_params, color_grid);

    sensor_msgs::PointCloud2 ros_pc2;
    open3dToRos(res_pcd, ros_pc2, data.header.frame_id);
    ros_pc2.header = data.points.header; // コレ大事
    _pub.publish(ros_pc2);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "open3d_test_hoge");

    ros::NodeHandle n_("~");

    // init subscribers and publishers
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/adapter/points_images", 1, onDataReceive);
    _pub = n.advertise<sensor_msgs::PointCloud2>("interpolated", 1);

    string params_name = "miyanosawa_3_3_rgb_original";
    cout << params_name << endl;
    params_use = loadParams(params_name);
    hyper_params = getDefaultHyperParams(params_use.isRGB);
    lidar_params = getDefaultLidarParams();

    // specify loop rate: a meaningful value according to your publisher configuration
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}