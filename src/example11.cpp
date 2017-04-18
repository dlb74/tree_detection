#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <boost/thread/thread.hpp>
//#include <pthread.h>

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <algorithm>
#include <string>
#include <math.h>


#include "mystuff.h"
#include "data.h"
#include "circle.h"
#include "Utilities.cpp"
#include "CircleFitByTaubin.cpp"
#include "CircleFitByLevenbergMarquardtFull.cpp"
#include "CircleFitByLevenbergMarquardtReduced.cpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <time.h>


/* La política de sincronización exacta NO FUNCIONA para los topics seleccionados, ya
 * que la Kinect o sus drivers no los están publicando con la misma marca temporal.
 * Ver más en http://wiki.ros.org/message_filters
 *
 * Exact sychronization policy IS NOT WORKING for the topics used, because the kinect
 * is not publishing them with the same timestamp.
 * See more in http://wiki.ros.org/message_filters  */

//#define EXACT
#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif



bool camera = true;

float scene_ss_ (0.02f);
float max_height (20.00f);


typedef pcl::PointXYZ PointT;

using namespace std;

using namespace message_filters;


// Contador para la numeración de los archivos.
// Counter for filenames.
unsigned int cnt = 1;



// Handler / callback

//void
//callback (const sensor_msgs::ImageConstPtr& depth_msg)
//{

//    cv_bridge::CvImagePtr img_ptr_rgb;
//    cv_bridge::CvImagePtr img_ptr_depth;

//    // Container for original & filtered data
//    try{

//        img_ptr_depth = cv_bridge::toCvCopy(*depth_msg, sensor_msgs::image_encoding::TYPE_16UC1);
//    }
//    catch(cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

//}

void callback( const sensor_msgs::ImageConstPtr& depth_msg )
{
        //ROS_INFO_STREAM("Adentro del callback\n");
      //cv_bridge::CvImagePtr img_ptr_rgb;
        cv_bridge::CvImagePtr img_ptr_depth;
    try{
        img_ptr_depth = cv_bridge::toCvCopy(*depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }


}





int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
  ros::init(argc, argv, "guardar_imagenes");
  ros::NodeHandle nh;


    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/camera/depth_registered/hw_registered/image_rect_raw" , 1 );
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/camera/rgb/image_rect_color" , 1 );


#ifdef EXACT
    typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif
#ifdef APPROXIMATE
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
#endif


  // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
  sync.registerCallback(boost::bind(&callback, _1));


    while(ros::ok())
    {
        char c;

        ROS_INFO_STREAM("\nIngrese 'a' para guardar un par de imágenes o 'b' para guardar 300 imágenes\n"
                        "Enter 'a' to save a pair of images or 'b' to automatically save 300 images\n");
        cin.get(c);
        cin.ignore();
        c = tolower(c);
        ROS_INFO_STREAM("You entered " << c << "\n");

        if( c == 'a' )
        {
                /* Le damos el control a la función callback cuando haya imágenes.
                * We give control to the callback function.*/
                ros::spinOnce();
        }

        else if( c == 'b' )
        {
            unsigned int cnt_init = cnt;
            while( cnt - cnt_init < 300 )
            {
                ros::spinOnce();
            }
        }

        else break;

    }
    ROS_INFO_STREAM("Cerrando nodo\nClosing node\n");

  return 0;
}
