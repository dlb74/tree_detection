
#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
class CameraInfo
{
public:
    CameraInfo(ros::NodeHandle nh);
    ~CameraInfo();
    ros::NodeHandle nh_;
    camera_info_manager::CameraInfoManager cinfo_mgr;
    sensor_msgs::CameraInfoPtr cinfo_msg;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::CameraPublisher camera_pub;
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};







#endif
