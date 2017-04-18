
#include <string>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>

#include "PCLTypeDefine.h"


using namespace std;

class PCLFilter
{
public:


    void downsample(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output);

    void outlierFilte(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output);

    double computeCloudResolution (const PointCloud::ConstPtr &cloud, int gap);

    void pointCloudSlice(vector< vector < PointCloud::Ptr > >& sliced_cloud,
                         vector<double>& sliceHeights,
                         PointCloud::Ptr cloud,
                         pcl::ModelCoefficients::Ptr plane_coefficient,
                         double resolution,
                         int sliceConst);

private:

};


void PCLFilter::downsample(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output)
{

    /**
    *  Downsample Clouds to Extract keypoints
    */
    pcl::UniformSampling<PointT> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_input);
    uniform_sampling.setRadiusSearch (scene_ss_);
    pcl::PointCloud<int> keypointIndices2;
    uniform_sampling.compute(keypointIndices2);
    pcl::copyPointCloud(*cloud_input, keypointIndices2.points, *cloud_output);
    //std::cout << "Scene total points: " << cloud_input->size () << "; Selected Keypoints: " << cloud_output->size () << std::endl;

}


void PCLFilter::outlierFilte(PointCloud::Ptr cloud_input, PointCloud::Ptr& cloud_output)
{
    /**
     * remove noisy measurements, e.g. outliers,
     * from a point cloud dataset using statistical
     * analysis techniques.
     */

    // Create the filtering object
    pcl::StatisticalOutlierRemoval< PointT > sor_outlier;
    sor_outlier.setInputCloud (cloud_input);
    sor_outlier.setMeanK (50);
    sor_outlier.setStddevMulThresh (1.0);
    sor_outlier.filter (*cloud_output);

//    std::cerr << "PointCloud after outlier filtering: " << cloud_output->width * cloud_output->height
//       << " data points"<<endl ;

}

double PCLFilter::computeCloudResolution (const PointCloud::ConstPtr &cloud, int gap)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    vector<int> indices (2);
    vector<float> sqr_distances (2);
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); i += cloud->size()/gap)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }

    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}


bool cloud_passthrough (PointCloud::Ptr cloud, PointCloud::Ptr cloud_out, double slice_height, double thickness)
{

    string axis = "z";
    // Create the filtering object
    pcl::PassThrough<PointT> pass;

    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    pass.setFilterLimits (slice_height, (slice_height + thickness));
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_out);

    //cout<<"cloud_out: "<<cloud_out->points.size()<<endl;

    if (!cloud_out->points.empty())
        return true;
    else
        return false;

}


void PCLFilter::pointCloudSlice(vector< vector < PointCloud::Ptr > >& sliced_cloud,
                     vector<double>& sliceHeights,
                     PointCloud::Ptr cloud,
                     pcl::ModelCoefficients::Ptr plane_coefficient,
                     double resolution,
                     int sliceConst = 1)
{

    double thickness = sliceConst * resolution;
    double currentHeight = -plane_coefficient->values[3] + 3;


    bool sliceScanned = false;

    while(1)
    {

        PointCloud::Ptr cloudUpperSlice(new PointCloud);
        PointCloud::Ptr cloudLowerSlice(new PointCloud);

        //currentHeight += step*thickness;
        //cout<<"eeeeeeeee: "<<currentHeight<<endl;
        if(cloud_passthrough (cloud, cloudUpperSlice, currentHeight, thickness) &&
            cloud_passthrough (cloud, cloudLowerSlice, currentHeight-(thickness), thickness))
        {
            sliceScanned = true;
            vector< PointCloud::Ptr> oneSlice;
            oneSlice.push_back(cloudLowerSlice);
            oneSlice.push_back(cloudUpperSlice);
            sliced_cloud.push_back(oneSlice);
            sliceHeights.push_back(currentHeight);
            oneSlice.clear();
            currentHeight -= sliceConst*thickness;

        }else{

            //cout<<"aaaaaaa: "<<fabs(currentHeight) <<"  "<<sliceScanned<<endl;
            if (sliceScanned || fabs(currentHeight) > max_height)
                break;
            else
                currentHeight -= sliceConst*thickness;
                continue;
        }
    }
}








