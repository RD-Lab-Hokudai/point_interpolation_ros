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
        float x = (*cloud)[i].x;
        float y = (*cloud)[i].y;
        float z = (*cloud)[i].z;

        float distance2 = x * x + y * y + z * z;
        if (distance2 <= 0.001)
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
    for (int i = 0; i < valid_cloud->size(); i++)
    {
        float x = (*valid_cloud)[i].x;
        float y = (*valid_cloud)[i].y;
        float z = (*valid_cloud)[i].z;

        float distance2 = x * x + y * y + z * z;

        //探索半径：係数*(距離)^2
        double radius = rad_coef * sqrt(distance2);

        vector<int> pointIdxNKNSearch;
        vector<float> pointNKNSquaredDistance;
        int result = kdtree->radiusSearch((*valid_cloud)[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance, min_k);
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