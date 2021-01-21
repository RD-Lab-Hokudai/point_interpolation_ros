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

void publish_cloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 10;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (int i = 0; i < 10; i++)
    {
        (*cloud)[i].x = i;
        (*cloud)[i].y = i;
        (*cloud)[i].z = i;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nan(new pcl::PointCloud<pcl::PointXYZ>);
    //vector<int> nan_index;
    //pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nan, nan_index);
    kdtree->setInputCloud(cloud);

    auto msg = cloud->makeShared();
    msg->header.frame_id = "os1_lidar";
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    _pub.publish(msg);
}

void onPointsReceive(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    double rad_coef = 0.001;
    int min_k = 2;
    int min_range = 10;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr near(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr far(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);
    int point_cnt = cloud->size();
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
    for (int i = 0; i < point_cnt; i++)
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
        cout << result << endl;
        if (result == min_k)
        {
            outliers->indices.push_back(i);
        }
    }
    cout << cloud->size() << endl;
    cout << outliers->indices.size() << endl;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(outliers);
    extract.setNegative(false);
    extract.filter(*cloud);
    cout << cloud->size() << endl;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    /*
    auto pub_msg = cloud->makeShared();
    pub_msg->header.frame_id = "os1_lidar";
    pcl_conversions::toPCL(ros::Time::now(), pub_msg->header.stamp);
    */
    _pub.publish(output);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kdtree_test_node");

    ros::NodeHandle n_("~");

    // init subscribers and publishers
    ros::NodeHandle n;
    _pub = n.advertise<sensor_msgs::PointCloud2>("dummy", 1);

    ros::Subscriber subPoints = n.subscribe("os1_cloud_node/points", 1, onPointsReceive);

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
        //publish_cloud();
        ros::spinOnce();
        loop_rate.sleep();
    }
}