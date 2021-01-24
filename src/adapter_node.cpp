#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <open3d_test/PointsImagesFront.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>

#include <Open3D/Open3D.h>

#include "models/lidarParams.cpp"
#include "io/rosToOpen3d.cpp"
#include "io/open3dToRos.cpp"
#include "preprocess/remove_noise2.cpp"

using namespace std;
using namespace open3d;

// Consider y coefficient
double k1 = 1.21456239e-05;
double k2 = 1.96249030e-14;
double k3 = 1.65216912e-05;
double k4 = 1.53712821e-11;
double p1 = -2.42560758e-06;
double p2 = -4.05806821e-06;
double alpha = 9.88930697e-01;
//int width = 882;
//int height = 560;
// Ignore p
int width = 938;
int height = 606;

cv::Mat thermal;
sensor_msgs::ImageConstPtr rgb;

ros::Publisher _pub_data;
LidarParams lidarParams;

void reverse_img(cv::Mat &src, cv::Mat &dst)
{
    dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    for (int i = 0; i < src.rows; i++)
    {
        uchar *dst_ptr = dst.ptr<uchar>(i);
        for (int j = 0; j < src.cols; j++)
        {
            dst_ptr[j] = src.at<uchar>(src.rows - i - 1, src.cols - j - 1);
        }
    }
}

void normalize_ushort2uchar(cv::Mat &src, cv::Mat &dst)
{
    dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    int maxVal = 0;
    int minVal = 66000;
    for (int i = 0; i < src.rows; i++)
    {
        for (int j = 0; j < src.cols; j++)
        {
            int val = src.at<ushort>(i, j);
            maxVal = max(maxVal, val);
            minVal = min(minVal, val);
        }
    }

    dst.forEach<uchar>([&](uchar &now, const int position[]) -> void {
        now = (uchar)((src.at<ushort>(position[0], position[1]) - minVal) * 255 / (maxVal - minVal));
    });
}

void correct_distortion(cv::Mat &src, cv::Mat &dst)
{
    dst = cv::Mat::zeros(height, width, CV_8UC1);
    for (int i = 0; i < height; i++)
    {
        uchar *out = dst.ptr<uchar>(i);
        for (int j = 0; j < width; j++)
        {
            double x1 = j - width / 2;
            double y1 = (height / 2 - i) * alpha;
            double r_2 = x1 * x1 + y1 * y1;
            double r_4 = r_2 * r_2;
            double x2 = x1 * (1 + k1 * r_2 + k2 * r_4) / (1 + k3 * r_2 + k4 * r_4); //+ 2 * p1 * x1 * y1 + p2 * (r_2 + 2 * x1 * x1);
            double y2 = y1 * (1 + k1 * r_2 + k2 * r_4) / (1 + k3 * r_2 + k4 * r_4); //+2 * p2 *x1 *y1 + p1 *(r_2 + 2 * y1 * y1);
            int ii = src.rows / 2 - (int)round(y2);
            int jj = src.cols / 2 + (int)round(x2);
            if (0 <= ii && ii < src.rows && 0 <= jj && jj < src.cols)
            {
                out[j] = src.at<uchar>(ii, jj);
            }
        }
    }
}

void onThermalReceive(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
    cv::Mat input8;
    normalize_ushort2uchar(input, input8);
    correct_distortion(input8, thermal);
}

void onRGBReceive(const sensor_msgs::ImageConstPtr &msg)
{
    rgb = msg;
}

void downsample_points(const sensor_msgs::PointCloud2 &src, sensor_msgs::PointCloud2 &dst)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(src, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < cloud->size(); i++)
    {
        double x = (*cloud)[i].x;
        double y = (*cloud)[i].y;
        double z = (*cloud)[i].z;

        double vertical_angle = atan2(z, sqrt(x * x + y * y)) * 180 / M_PI;
        int rowIdx = (int)((vertical_angle + lidarParams.bottom_angle) / lidarParams.vertical_res + 0.5);
        if (rowIdx >= 0 && rowIdx < lidarParams.height && rowIdx % 4 == 0)
        {
            result->push_back(pcl::PointXYZ(x, y, z));
        }
    }

    pcl::toROSMsg(*result, dst);
    dst.header.frame_id = src.header.frame_id;
}

void onPointsReceive(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!rgb || !thermal.data)
    {
        return;
    }

    open3d_test::PointsImagesFront pub_msg;
    pub_msg.thermal = *cv_bridge::CvImage(std_msgs::Header(), "mono8", thermal).toImageMsg();
    pub_msg.rgb = *rgb;
    sensor_msgs::PointCloud2 downsampled;
    downsample_points(*msg, downsampled);
    //downsampled = *msg;
    sensor_msgs::PointCloud2 output;
    //remove_noise2(downsampled, output, 0.001, 2);
    remove_noise2(downsampled, output, 0.008, 2);
    pub_msg.points = output;
    _pub_data.publish(pub_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kdtree_test_node");

    ros::NodeHandle n_("~");

    // init subscribers and publishers
    ros::NodeHandle n;

    lidarParams = getDefaultLidarParams();

    string thermal_node, rgb_node, points_node;
    n.param<std::string>("thermal_node", thermal_node, "/thermal_image");
    n.param<std::string>("rgb_node", rgb_node, "/rgb_image");
    n.param<std::string>("points_node", points_node, "/os1_cloud_node/points");
    cout << thermal_node << endl;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber subThermal = it.subscribe(thermal_node, 1, onThermalReceive);
    image_transport::Subscriber subRGB = it.subscribe(rgb_node, 1, onRGBReceive);

    ros::Subscriber subPoints = n.subscribe(points_node, 1, onPointsReceive);

    _pub_data = n.advertise<open3d_test::PointsImagesFront>("/adapter/points_images_front", 1);

    // specify loop rate: a meaningful value according to your publisher configuration
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        //publish_cloud();
        ros::spinOnce();
        loop_rate.sleep();
    }
}