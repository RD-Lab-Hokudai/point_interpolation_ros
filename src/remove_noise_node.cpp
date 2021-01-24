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

#include "preprocess/remove_noise2.cpp"

using namespace std;

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

void onPointsReceive(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 output;
    remove_noise2(*msg, output, 0.01, 3);
    _pub_removed.publish(output);
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

    //ros::Subscriber subData = n.subscribe("/adapter/points_images_front", 1, onDataReceive);

    // specify loop rate: a meaningful value according to your publisher configuration
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        //publish_cloud();
        ros::spinOnce();
        loop_rate.sleep();
    }
}