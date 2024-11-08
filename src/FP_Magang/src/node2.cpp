#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "FP_Magang/PC2BS.h"

ros::Publisher pos_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat image;
    try {
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert BGR to HSV
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Threshold the HSV image to get only orange colors
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(5, 100, 100), cv::Scalar(15, 255, 255), mask);

    // Reduce noise in the mask
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    FP_Magang::PC2BS ball_msg;
    ball_msg.bola_x = -1;
    ball_msg.bola_y = -1;

    if (!contours.empty()) {
        // Assume the largest contour is the ball
        auto max_contour = std::max_element(contours.begin(), contours.end(),
                                            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                                                return cv::contourArea(a) < cv::contourArea(b);
                                            });

        // Get the ball position
        cv::Moments M = cv::moments(*max_contour);
        if (M.m00 != 0) {
            ball_msg.bola_x = M.m10 / M.m00;
            ball_msg.bola_y = M.m01 / M.m00;
        }
    }

    pos_pub.publish(ball_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processing");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/image_topic", 10, imageCallback);
    pos_pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 10);

    ros::spin();
    return 0;
}
