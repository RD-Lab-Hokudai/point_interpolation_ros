#include <math.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

using namespace std;

image_transport::Publisher _pub;
/*
double k1 = 1.05791597e-06;
double k2 = 5.26154073e-14;
double k3 = 3.41991153e-06;
double k4 = 3.27612688e-13;
double p1 = -4.30326545e-06;
double p2 = -4.60648477e-06;
double alpha = 1.0;
int width = 882;
int height = 560;
*/

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

void onDataReceive(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat input = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
        cv::Mat img = cv::Mat::zeros(height, width, CV_8UC1);
        int maxVal = 0;
        int minVal = 66000;
        for (int i = 0; i < input.rows; i++)
        {
            for (int j = 0; j < input.cols; j++)
            {
                int val = input.at<ushort>(i, j);
                maxVal = max(maxVal, val);
                minVal = min(minVal, val);
            }
        }

        for (int i = 0; i < img.rows; i++)
        {
            uchar *out = img.ptr<uchar>(i);
            for (int j = 0; j < img.cols; j++)
            {
                double x1 = j - img.cols / 2;
                double y1 = (img.rows / 2 - i) * alpha;
                double r_2 = x1 * x1 + y1 * y1;
                double r_4 = r_2 * r_2;
                double x2 = x1 * (1 + k1 * r_2 + k2 * r_4) / (1 + k3 * r_2 + k4 * r_4); //+ 2 * p1 * x1 * y1 + p2 * (r_2 + 2 * x1 * x1);
                double y2 = y1 * (1 + k1 * r_2 + k2 * r_4) / (1 + k3 * r_2 + k4 * r_4); //+2 * p2 *x1 *y1 + p1 *(r_2 + 2 * y1 * y1);
                int ii = input.rows / 2 - (int)round(y2);
                int jj = input.cols / 2 + (int)round(x2);
                if (0 <= ii && ii < input.rows && 0 <= jj && jj < input.cols)
                {
                    uchar val = (uchar)((input.at<ushort>(ii, jj) - minVal) * 255 / (maxVal - minVal));
                    out[j] = val;
                }
            }
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, img).toImageMsg();
        _pub.publish(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_beidge exception: %s", e.what());
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "optris_correct_distortion");

    ros::NodeHandle n_("~");

    // init subscribers and publishers
    ros::NodeHandle n;
    ros::Subscriber subThermal = n.subscribe("/thermal_image", 1, onDataReceive);
    image_transport::ImageTransport it(n);
    _pub = it.advertise("/thermal_corrected", 1);

    // specify loop rate: a meaningful value according to your publisher configuration
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
