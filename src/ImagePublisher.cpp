#include "ImagePublisher.h"
using namespace std;
CameraInfo::CameraInfo(ros::NodeHandle nh):
    nh_(nh),
    cinfo_mgr(nh_,"mv_26804026","file:///home/xianyu/.ros/camera_info/mv_26804026.yaml"),
    it(nh_),
    camera_pub(it.advertiseCamera("mv_26804026/image_raw",1)),
    image_sub(it.subscribe("/mv_26804026/image_",1,&CameraInfo::imageCallback, this)),
    cinfo_msg(boost::make_shared<sensor_msgs::CameraInfo>(cinfo_mgr.getCameraInfo()))
{
}
CameraInfo::~CameraInfo()
{
    ROS_INFO("Destroying CameraInfo...");
}

void CameraInfo::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cinfo_msg->header = msg->header;
    camera_pub.publish(msg, cinfo_msg);
}
