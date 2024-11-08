#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/image_topic", 10);

    // Load image
    cv::Mat image = cv::imread("/home/rezaafzaal/Downloads/bola2.jpg");
    if (image.empty()) {
        ROS_ERROR("Failed to load image!");
        return -1;
    }

    // Resize image to 900x600
    cv::resize(image, image, cv::Size(900, 600));

    ros::Rate loop_rate(5);
    while (ros::ok()) {
        // Convert OpenCV image to ROS Image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
