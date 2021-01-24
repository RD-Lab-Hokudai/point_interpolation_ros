#pragma once
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

void remove_noise2(const sensor_msgs::PointCloud2 &src, sensor_msgs::PointCloud2 &dst, double rad_coef = 0.01, int min_k = 2)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(src, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr valid_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < cloud->size(); i++)
    {
        double x = (*cloud)[i].x;
        double y = (*cloud)[i].y;
        double z = (*cloud)[i].z;

        double distance2 = x * x + y * y + z * z;
        if (distance2 <= 0.1)
        {
        }
        else
        {
            valid_cloud->push_back((*cloud)[i]);
        }
    }

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(valid_cloud);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    vector<int> pointIdxNKNSearch;
    vector<float> pointNKNSquaredDistance;
    for (int i = 0; i < valid_cloud->size(); i++)
    {
        double x = valid_cloud->points[i].x;
        double y = valid_cloud->points[i].y;
        double z = valid_cloud->points[i].z;
        double distance2 = x * x + y * y + z * z;

        //探索半径：係数*(距離)^2
        double radius = rad_coef * distance2;

        int result = kdtree->radiusSearch(cloud->points[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance, min_k);
        if (result == min_k)
        {
            inliers->indices.push_back(i);
        }
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(valid_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*valid_cloud);

    pcl::toROSMsg(*valid_cloud, dst);
    dst.header.frame_id = src.header.frame_id;
}