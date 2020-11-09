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
#include "data/loadParams.cpp"
#include "interpolate.cpp"

using namespace std;
using namespace open3d;

EnvParams params_use;
HyperParams hyper_params;

ros::Publisher _pub;

/*
shared_ptr<geometry::PointCloud> downsample(geometry::PointCloud &pcd, EnvParams &envParams)
{
    vector<double> tans;
    double PI = acos(-1);
    double delta_rad = 0.52698 * PI / 180;
    double max_rad = (16.6 + 0.26349) * PI / 180;
    double rad = (-16.6 + 0.26349) * PI / 180;
    while (rad < max_rad + 0.00001)
    {
        tans.emplace_back(tan(rad));
        rad += delta_rad;
    }

    vector<vector<Eigen::Vector3d>> all_layers(64, vector<Eigen::Vector3d>());
    double rollVal = (envParams.roll - 500) / 1000.0;
    double pitchVal = (envParams.pitch - 500) / 1000.0;
    double yawVal = (envParams.yaw - 500) / 1000.0;
    Eigen::MatrixXd calibration_mtx(3, 3);
    calibration_mtx << cos(yawVal) * cos(pitchVal), cos(yawVal) * sin(pitchVal) * sin(rollVal) - sin(yawVal) * cos(rollVal), cos(yawVal) * sin(pitchVal) * cos(rollVal) + sin(yawVal) * sin(rollVal),
        sin(yawVal) * cos(pitchVal), sin(yawVal) * sin(pitchVal) * sin(rollVal) + cos(yawVal) * cos(rollVal), sin(yawVal) * sin(pitchVal) * cos(rollVal) - cos(yawVal) * sin(rollVal),
        -sin(pitchVal), cos(pitchVal) * sin(rollVal), cos(pitchVal) * cos(rollVal);

    for (int i = 0; i < pcd.points_.size(); i++)
    {
        double rawX = pcd.points_[i][1];
        double rawY = -pcd.points_[i][2];
        double rawZ = -pcd.points_[i][0];

        double r = sqrt(rawX * rawX + rawZ * rawZ);
        double xp = calibration_mtx(0, 0) * rawX + calibration_mtx(0, 1) * rawY + calibration_mtx(0, 2) * rawZ;
        double yp = calibration_mtx(1, 0) * rawX + calibration_mtx(1, 1) * rawY + calibration_mtx(1, 2) * rawZ;
        double zp = calibration_mtx(2, 0) * rawX + calibration_mtx(2, 1) * rawY + calibration_mtx(2, 2) * rawZ;
        double x = xp + (envParams.X - 500) / 100.0;
        double y = yp + (envParams.Y - 500) / 100.0;
        double z = zp + (envParams.Z - 500) / 100.0;

        auto it = lower_bound(tans.begin(), tans.end(), rawY / r);
        int index = it - tans.begin();
        all_layers[index].emplace_back(x, y, z);
    }

    auto res_ptr = make_shared<geometry::PointCloud>();
    for (int i = 0; i < all_layers.size(); i += 1)
    {
        for (int j = 0; j < all_layers[i].size(); j++)
        {
            double x = -all_layers[i][j][2];
            double y = all_layers[i][j][0];
            double z = -all_layers[i][j][1];
            res_ptr->points_.emplace_back(x, y, z);
        }
    }

    return res_ptr;
}

void onDataReceive(const sensor_msgs::PointCloud2ConstPtr &data)
{
    open3d::geometry::PointCloud pcd;
    rosToOpen3d(data, pcd);

    auto res_ptr = downsample(pcd, params_use);

    sensor_msgs::PointCloud2 ros_pc2;
    open3dToRos(*res_ptr, ros_pc2, data->header.frame_id);
    ros_pc2.header = data->header;
    _pub.publish(ros_pc2);
}
*/

void onDataReceive(const open3d_test::PointsImages &data)
{
    cv::Mat rgb_front = cv_bridge::toCvCopy(data.rgb_front)->image;

    open3d::geometry::PointCloud pcd;
    rosToOpen3d(data.points, pcd); // pcdへの変換がおかしい？

    auto res = interpolate(rgb_front, pcd, params_use, hyper_params);
    open3d::geometry::PointCloud hoge;
    for (int i = 0; i < pcd.points_.size(); i++)
    {
        double x = pcd.points_[i][0];
        double y = pcd.points_[i][1];
        double z = pcd.points_[i][2];
        if (z > 0 && abs(x / z) < 1.2)
        {
            hoge.points_.emplace_back(x, y, z);
        }
    }

    sensor_msgs::PointCloud2 ros_pc2;
    open3dToRos(hoge, ros_pc2, data.header.frame_id);
    ros_pc2.header = data.points.header;
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

    // specify loop rate: a meaningful value according to your publisher configuration
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}