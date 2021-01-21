#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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

void onPointsReceive(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    double rad_coef = 0.01;
    int min_k = 2;
    int min_range = 3;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr near(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr far(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < cloud->size(); i++)
    {
        double x = (*cloud)[i].x;
        double y = (*cloud)[i].y;
        double z = (*cloud)[i].z;

        double distance = x * x + y * y + z * z;
        if (distance <= min_range * min_range)
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
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
    for (int i = 0; i < near->size(); i++)
    {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
        double z = cloud->points[i].z;
        double distance2 = x * x + y * y + z * z;

        //探索半径：係数*(距離)^2
        double radius = rad_coef * distance2;

        vector<int> pointIdxNKNSearch(min_k);
        vector<float> pointNKNSquaredDistance(min_k);

        int result = kdtree->radiusSearch(cloud->points[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance, min_k);
        if (result == min_k)
        {
            outliers->indices.push_back(i);
        }
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(near);
    extract.setIndices(outliers);
    extract.setNegative(false);
    extract.filter(*near);

    *near += *far;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*near, output);
    output.header.frame_id = "os1_lidar";
    _pub_removed.publish(output);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kdtree_test_node");

    ros::NodeHandle n_("~");

    // init subscribers and publishers
    ros::NodeHandle n;
    _pub = n.advertise<sensor_msgs::PointCloud2>("aaa/points", 1);
    _pub_removed = n.advertise<sensor_msgs::PointCloud2>("aaa/removed", 1);

    ros::Subscriber subPoints = n.subscribe("aaa/points", 1, onPointsReceive);

    /*
    auto ptr = make_shared<geometry::PointCloud>();
    ptr->points_.emplace_back(1, 1, 1);
    ptr->points_.emplace_back(10, 10, 10);
    ptr->points_.emplace_back(100, 100, 100);
    auto kdtree = make_shared<geometry::KDTreeFlann>(*ptr);
    */

    // specify loop rate: a meaningful value according to your publisher configuration
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        publish_cloud();
        ros::spinOnce();
        loop_rate.sleep();
    }
}