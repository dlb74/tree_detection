
#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <unistd.h>

#include "PCLFilter.h"


using namespace std;
using namespace cv;

pcl::visualization::CloudViewer viewer("Cloud Viewer");

PCLFilter filter;

bool cb_flag;

void cloudProcessing(const PointCloud::ConstPtr& cloud_in, PointCloud::Ptr& cloud_out)
{

    PointCloud::Ptr cloud_out_downsample (new PointCloud);
    PointCloud::Ptr cloud_out_outlier (new PointCloud);

    cloud_out_downsample = cloud_in->makeShared();

    /**
    * use downsampling filter
    */
    filter.downsample(cloud_out_downsample, cloud_out_outlier);

    /**
    * outlier filter
    */
    filter.outlierFilte(cloud_out_outlier, cloud_out);

    double resolution = filter.computeCloudResolution(cloud_out, 100);
    cout<<"resolution: "<< resolution<<endl;

    //point cloud slice
    vector<vector < PointCloud::Ptr > > slicedCloud3d;
    vector < PointCloud::Ptr > slicedCloud2d;
    vector<double> sliceHeights;

    //filter.pointCloudSlice(slicedCloud3d, sliceHeights, cloud_out, plane_coefficient, resolution, 2);

}


/**
 * Cloud point call back from subscriber
 */
void cloud_cb(const PointCloud::ConstPtr& cloud_msg)
{
    try{
    
        cb_flag = false;

        PointCloud::Ptr scene (new PointCloud);
        cloudProcessing(cloud_msg, scene);
        viewer.showCloud(scene);
        
        cb_flag = true;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::Mat& mat_depth = img_ptr_depth->image;
}


/**
 * Depth image call back from subscriber
 */
void depth_cb(const sensor_msgs::ImageConstPtr& depth_msg)
{

    cv_bridge::CvImagePtr img_ptr_depth;

    // Container for original & filtered data
    try{

        img_ptr_depth = cv_bridge::toCvCopy(*depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat& mat_depth = img_ptr_depth->image;

    Mat image;
//    image = imread(*mat_depth);   // Read the file

//    if(! image.data )                              // Check for invalid input
//    {
//        cout <<  "Could not open or find the image" << std::endl ;

//    }

//    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//    cv::imshow( "Display window", image );

//    cv::waitKey(0);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "cloud_listener");

    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    //ros::Subscriber sub = n.subscribe ("/camera/depth/image_raw", 1000, depth_cb);

    ros::Subscriber sub_cloud = n.subscribe<PointCloud> ("/camera/depth_registered/points", 1, cloud_cb);

    ros::Subscriber sub_depth = n.subscribe ("/camera/depth/image_raw", 1, depth_cb);
    
    cb_flag = true;

    ros::Rate r(10);

    while (ros::ok())
    {

        cout<<"cb_flag: "<<cb_flag<<endl;
        if (cb_flag == true)
        {
            ros::spinOnce();                   // Handle ROS events
            r.sleep();
        }

    }


    return 0;
}
