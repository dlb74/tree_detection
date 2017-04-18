#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ImagePublisher.h>




void chatterCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    //ROS_INFO("I heard: [%s]", depth_msg->data.c_str());

    cv_bridge::CvImagePtr img_ptr_rgb;
    cv::Mat img_depth;
    sensor_msgs::ImagePtr img_ptr_depth;

    // Container for original & filtered data
    try{

      img_depth = cv_bridge::toCvShare(depth_msg, "bgr8")->image;
//      cv::imshow("view", img_depth);
//      cv::waitKey(30);
//      img_ptr_depth = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_depth).toImageMsg();

    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


    sensor_msgs::CameraInfo info_camera;


}

void publish(const sensor_msgs::ImagePtr &image)
{
    image->header.frame_id = "camera_rgb_optical_frame";

    ros::NodeHandle nh;
    // get current CameraInfo data
//    camera_info_manager::CameraInfoManager *cim = new camera_info_manager::CameraInfoManager(nh);
//    *cim = new camera_info_manager::CameraInfoManager(nh);
//    sensor_msgs::CameraInfoPtr
//      ci(new sensor_msgs::CameraInfo(cim->getCameraInfo()));

//    // check whether CameraInfo matches current video mode
//    if (!dev_->checkCameraInfo(*image, *ci))
//      {
//        // image size does not match: publish a matching uncalibrated
//        // CameraInfo instead
//        if (calibration_matches_)
//          {
//            // warn user once
//            calibration_matches_ = false;
//            ROS_WARN_STREAM("[" << camera_name_
//                            << "] calibration does not match video mode "
//                            << "(publishing uncalibrated data)");
//          }
//        ci.reset(new sensor_msgs::CameraInfo());
//        ci->height = image->height;
//        ci->width = image->width;
//      }
//    else if (!calibration_matches_)
//      {
//        // calibration OK now
//        calibration_matches_ = true;
//        ROS_WARN_STREAM("[" << camera_name_
//                        << "] calibration matches video mode now");
//      }

//    // fill in operational parameters
//    dev_->setOperationalParameters(*ci);

//    ci->header.frame_id = config_.frame_id;
//    ci->header.stamp = image->header.stamp;

//    // @todo log a warning if (filtered) time since last published
//    // image is not reasonably close to configured frame_rate

//    // Publish via image_transport
//    image_pub_.publish(image, ci);
}



int main(int argc, char **argv)
{
    //boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;


    ros::init(argc, argv, "listener");


    ros::NodeHandle nh;


    ros::Subscriber sub = nh.subscribe("/rgb_camera_0depth_range_mm", 1000, chatterCallback);


    //CameraInfo CCameraInfo(nh);

    ros::spin();

    return 0;
}








