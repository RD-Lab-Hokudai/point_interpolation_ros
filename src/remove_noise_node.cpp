#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <open3d_test/PointsImagesFront.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>

#include <Open3D/Open3D.h>

using namespace std;
using namespace open3d;

ros::Publisher _pub;
ros::Publisher _pub_removed;
ros::Publisher _pub_data;

void publish_cloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 10000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (int i = 0; i < cloud->size(); i++)
    {
        double len = i * i * 0.00000005;
        double rad = i * 0.01;
        (*cloud)[i].x = len * cos(rad);
        (*cloud)[i].y = 0;
        (*cloud)[i].z = len * sin(rad);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "os1_lidar";
    _pub.publish(output);
}

void remove_noise(const sensor_msgs::PointCloud2 &src, sensor_msgs::PointCloud2 &dst, double rad_coef = 0.01, int min_k = 2)
{
    int min_range = 3;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(src, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr near(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr far(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < cloud->size(); i++)
    {
        double x = (*cloud)[i].x;
        double y = (*cloud)[i].y;
        double z = (*cloud)[i].z;

        double distance2 = x * x + y * y + z * z;
        if (distance2 <= min_range * min_range)
        {
            near->push_back((*cloud)[i]);
        }
        else
        {
            far->push_back((*cloud)[i]);
        }
    }

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

        int result = kdtree->radiusSearch(cloud->points[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance, min_k);
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
    pcl::toROSMsg(*near, dst);
    dst.header.frame_id = src.header.frame_id;
}

void onPointsReceive(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 output;
    remove_noise(*msg, output, 0.01, 3);
    _pub_removed.publish(output);
}

void onDataReceive(const open3d_test::PointsImagesFront &data)
{
    open3d_test::PointsImagesFront pub_msg;
    pub_msg.thermal = data.thermal;
    pub_msg.rgb = data.rgb;
    //sensor_msgs::PointCloud2 output;
    //remove_noise(data.points, output);
    //pub_msg.points = output;
    pub_msg.points = data.points;
    _pub_data.publish(pub_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kdtree_test_node");

    ros::NodeHandle n_("~");

    // init subscribers and publishers
    ros::NodeHandle n;
    _pub = n.advertise<sensor_msgs::PointCloud2>("aaa/points", 1);
    _pub_removed = n.advertise<sensor_msgs::PointCloud2>("/removed", 1);
    _pub_data = n.advertise<open3d_test::PointsImagesFront>("/removed/points_images_front", 1);

    ros::Subscriber subPoints = n.subscribe("/interpolated", 1, onPointsReceive);

    ros::Subscriber subData = n.subscribe("/adapter/points_images_front", 1, onDataReceive);

    // specify loop rate: a meaningful value according to your publisher configuration
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        //publish_cloud();
        ros::spinOnce();
        loop_rate.sleep();
    }
}