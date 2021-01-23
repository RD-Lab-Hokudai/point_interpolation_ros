#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <open3d_test/PointsImagesFront.h>

/*
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
*/

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

void interpolate_original_thermal(vector<vector<double>> &grid, cv::Mat &thermal, vector<vector<Eigen::Vector3d>> &color_grid)
{
    original_entire(grid, params_use, hyper_params, lidar_params, thermal, 0, color_grid);
}

void onDataReceive(const open3d_test::PointsImagesFront &data)
{
    cv::Mat thermal = cv_bridge::toCvCopy(data.thermal)->image;
    cv::Mat rgb_front = cv_bridge::toCvCopy(data.rgb)->image;

    //open3d::geometry::PointCloud pcd;
    //rosToOpen3d(data.points, pcd);

    //auto res_pcd = interpolate(pcd, rgb_front, rgb_right, rgb_back, rgb_left);
    vector<vector<double>> grid;
    grid_entire2(data.points, grid, lidar_params);
    vector<vector<Eigen::Vector3d>> color_grid(lidar_params.height, vector<Eigen::Vector3d>(lidar_params.width, Eigen::Vector3d(0, 0, 0)));
    interpolate_original_thermal(grid, thermal, color_grid);
    //interpolate_original4(grid, rgb_front, rgb_right, rgb_back, rgb_left, color_grid);
    //original_entire(grid, params_use, hyper_params, lidar_params, rgb_right, -lidar_params.width / 4, color_grid);
    //linear_entire(grid, lidar_params);
    //auto res_pcd = create_cloud_entire(grid, lidar_params, color_grid);

    sensor_msgs::PointCloud2 ros_pc2;
    //open3dToRos(res_pcd, ros_pc2, data.points.header.frame_id);
    create_cloud_entire2(grid, lidar_params, ros_pc2);
    _pub.publish(ros_pc2);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "open3d_test_interpolate");

    ros::NodeHandle n_("~");

    // init subscribers and publishers
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/adapter/points_images_front", 1, onDataReceive);
    _pub = n.advertise<sensor_msgs::PointCloud2>("interpolated", 1);

    string params_name = "miyanosawa_3_3_thermal_original";
    //"miyanosawa_1203_thermal_original";
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