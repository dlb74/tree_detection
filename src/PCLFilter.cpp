#include "PCLFilter.h"


//void PCLFilter::hehe()
//{
//    std::cout<<"hehe";
//}

//void PCLFilter::downsample(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output)
//{

//    /**
//    *  Downsample Clouds to Extract keypoints
//    */
//    pcl::UniformSampling<PointT> uniform_sampling;
//    uniform_sampling.setInputCloud (cloud_input);
//    uniform_sampling.setRadiusSearch (0.02f);
//    pcl::PointCloud<int> keypointIndices2;
//    uniform_sampling.compute(keypointIndices2);
//    pcl::copyPointCloud(*cloud_input, keypointIndices2.points, *cloud_output);
//    //std::cout << "Scene total points: " << cloud_input->size () << "; Selected Keypoints: " << cloud_output->size () << std::endl;

//}
