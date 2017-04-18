#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <boost/thread/thread.hpp>
//#include <pthread.h>

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_visualizer.h"
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
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <algorithm>
#include <string>
#include <math.h>

#include "PCLTypeDefine.h"
#include "mystuff.h"
#include "data.h"
#include "circle.h"
#include "Utilities.cpp"
#include "CircleFitByTaubin.cpp"
#include "CircleFitByLevenbergMarquardtFull.cpp"
#include "CircleFitByLevenbergMarquardtReduced.cpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <time.h>





bool camera = false;
bool initialized = false;
bool segmentForest = false;



//scan point cloud from bottom to top. 0 = ground. 1 = tree trunk. 2 = tree crown area.

int atGround (0);
int atTrunkAssume(1);
int atTrunk (2);
int atCrownAssume(3);
int atCrown (4);



using namespace std;

using namespace message_filters;

struct CircleData
{
	cv::Point2f center;
    double radius;
    double height;
};

ros::Publisher pub;
pcl::PCDWriter writer;

int currentStatus = atGround;

int rows = 400;
int cols = 400;
int zoomSize = 50;
int cout_n = 0;

float min_radius = 0.02;
float max_radius = 3.0;

float slice_step = 0.1;
int step = 3;

string axis = "z";

//pcl::visualization::CloudViewer viewer ("3D Cloud");
//boost::shared_ptr<pcl::visualization::PCLVisualizer> my_viewer;


double computeCloudResolution (const PointCloud::ConstPtr &cloud, int gap)
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




//float zoomin (float input)
//{
//	return (rows/4) + input * zoomSize;
//}

//float zoomout (float input)
//{
//	return (input - (rows/4)) / zoomSize;
//}

bool clockCompare(PointT a, PointT b)
{
    PointT center;
    center.x = 0;
    center.y = 0;
    if (a.x - center.x >= 0 && b.x - center.x < 0)
        return true;
    if (a.x - center.x < 0 && b.x - center.x >= 0)
        return false;
    if (a.x - center.x == 0 && b.x - center.x == 0) {
        if (a.y - center.y >= 0 || b.y - center.y >= 0)
            return a.y > b.y;
        return b.y > a.y;
    }

    // compute the cross product of vectors (center -> a) x (center -> b)
    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;

    // points a and b are on the same line from the center
    // check which point is closer to the center
    int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
    int d2 = (b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y);
    return d1 > d2;
}

void bubbleSort(PointCloud::Ptr cloud, int size)
{
	PointT temp;
	for(int pass=1;pass<size;pass++)
	{
        for(int k=0;k<size-pass;k++)
            //clockCompare(cloud->points[k], cloud->points[k+1])
            if(clockCompare(cloud->points[k], cloud->points[k+1]))
			{
				temp=cloud->points[k];
				cloud->points[k]=cloud->points[k+1];
				cloud->points[k+1]=temp;       
			}
	}
}

float slope_weight_cal(cv::Point2f p1, cv::Point2f p2, int weight){
	//cout << "dian: "<< p1 <<" "<< p2 << endl;
	
	if (p2.x != p1.x){
		float distance = (float)sqrt((p2.y-p1.y)*(p2.y-p1.y) + (p2.x-p1.x)*(p2.x-p1.x));
		float slope = (float)(p2.y-p1.y)/(p2.x-p1.x);
		return (float) (slope*(1+(weight*distance)));
	}
	else
		return 0.0;
}
double random(double start, double end)
{
	return start+(end-start)*rand()/(RAND_MAX+1.0);
}


CircleData findCircleCenterLeast(PointCloud::Ptr linePCL)
{
	vector<cv::Point2f> line;
    for (int i = 0; i < linePCL->points.size(); i++)
    {
        cv::Point2f tempPoint (cv::Point2f(linePCL->points[i].x, linePCL->points[i].y));
        line.push_back(tempPoint);
    }

    int code;

    reals BenchmarkExampleDataX[line.size()];
    reals BenchmarkExampleDataY[line.size()];

    for (int i = 0; i < line.size(); i++)
    {
        BenchmarkExampleDataX[i] = line[i].x;
        BenchmarkExampleDataY[i] = line[i].y;
    }
    
    reals LambdaIni=0.001;

	Data data1(line.size(),BenchmarkExampleDataX,BenchmarkExampleDataY);
    Circle circle,circleIni;
    cout.precision(7);
    
    circleIni = CircleFitByTaubin (data1);

    code = CircleFitByLevenbergMarquardtFull (data1,circleIni,LambdaIni,circle);


    CircleData CD;

    CD.center.x = circle.a;
    CD.center.y = circle.b;
    //get radius
    CD.radius = circle.r;
    CD.height = linePCL->points[0].z;



    return CD;


}

bool isClosePoint2D(double pt1x, double pt1y, double pt2x, double pt2y, double threshold = 0.4)
{

    if(sqrtf(fabs(pt1x-pt2x)*fabs(pt1x-pt2x)+fabs(pt1y-pt2y)*fabs(pt1y-pt2y)) < threshold)
    {
        //cout<<"heheda: "<<sqrtf(fabs(pt1x-pt2x)*fabs(pt1x-pt2x)+fabs(pt1y-pt2y)*fabs(pt1y-pt2y))<<endl;
        return true;
    }
    return false;
}



CircleData findCircleCenter(pcl::PointCloud<PointT>::Ptr line, int repeat = 10000)
{

	//cv::Point2f pt1 = line[int(random(0, line.size()))];
	//cv::Point2f pt2 = line[int(random(0, line.size()))];
	//cv::Point2f pt3 = line[int(random(0, line.size()))];
    vector<CircleData> circles;
    int repeat_weight = 0;
    CircleData CD;
    for (int i = 0; i < repeat; i++)
    {
        cv::Point2f pt1;
        pt1.x = line->points[rand() % line->points.size()].x;
        pt1.y = line->points[rand() % line->points.size()].y;
        cv::Point2f pt2;
        pt2.x = line->points[rand() % line->points.size()].x;
        pt2.y = line->points[rand() % line->points.size()].y;
        cv::Point2f pt3;
        pt3.x = line->points[rand() % line->points.size()].x;
        pt3.y = line->points[rand() % line->points.size()].y;

        //two mid point
        cv::Point2f midpt1, midpt2;
        midpt1.x = (pt2.x + pt1.x) / 2;
        midpt1.y = (pt2.y + pt1.y) / 2;
        //find mid point
        midpt2.x = (pt3.x + pt1.x) / 2;
        midpt2.y = (pt3.y + pt1.y) / 2;
        //find slopes of mid perpendicular of pt1pt2，pt1pt3
        float k1 = -(pt2.x - pt1.x) / (pt2.y - pt1.y);
        float k2 = -(pt3.x - pt1.x) / (pt3.y - pt1.y);



        CD.center.x = (midpt2.y - midpt1.y - k2* midpt2.x + k1*midpt1.x) / (k1 - k2);
        CD.center.y = midpt1.y + k1*(midpt2.y - midpt1.y - k2*midpt2.x + k2*midpt1.x) / (k1 - k2);
        //get radius
        CD.radius = sqrtf((CD.center.x - pt1.x)*(CD.center.x - pt1.x) + (CD.center.y - pt1.y)*(CD.center.y - pt1.y));
        CD.height = line->points[0].z;

        circles.push_back(CD);

        if (repeat > 1)
        {

            if (isClosePoint2D(circles[i-1].center.x, circles[i-1].center.y, circles[i].center.x, circles[i].center.y, CD.radius/4))
            {
                CD.center.x = (circles[i-1].center.x + circles[i].center.x)/2;
                CD.center.y = (circles[i-1].center.y + circles[i].center.y)/2;
                CD.radius = (circles[i-1].radius + circles[i].radius)/2;
                repeat_weight++;

                //cout<<"CD.center: "<<CD.center.x<<"  "<<CD.center.y<<endl;
            }
            if (repeat_weight > (repeat/2))
            {
                return CD;
            }

        }

    }

	return CD;

}



CircleData findCircleCenter1(pcl::PointCloud<PointT>::Ptr line)
{

    int i;
    int j;
    int k;

    int points_size = line->points.size();

    vector< vector<double> > A(3);
    vector< vector<double> > Y(1);
    vector< vector<double> > transA(points_size);
    vector< vector<double> > multATA(points_size);
    vector< vector<double> > multATY(points_size);
    //double A[3][points_size];
//    double Y[1][points_size];
//    double transA[points_size][3];
//    double multATA[points_size][1000000];
//    cout<<"111111111111111111: " <<endl;
//    // setup the matrices


    for (j=0; j < points_size; j++)
    {
        double x = line->points[j].x;
        double y = line->points[j].y;

        A[0].push_back(x);
        A[1].push_back(y);
        A[2].push_back(1);
    }

    for (i=0; i < points_size; i++)
    {
        Y[0].push_back(line->points[i].x * line->points[i].x + line->points[i].y * line->points[i].y);
    }

    //transpose matrix A
    for(i = 0; i < 3; ++i)
    {

        for(j = 0; j < line->points.size(); ++j)
        {
            transA[j].push_back(A[i][j]);
        }
    }

    //multiply transposed A and A
    for(i = 0; i < line->points.size(); ++i)
    {
        multATA[i].resize(points_size);
        multATY[i].resize(points_size);
    }


    for(i = 0; i < line->points.size(); ++i)
    {
        for(j = 0; j < line->points.size(); ++j)
        {
            for(k = 0; k < 3; ++k)
            {
                multATA[i][j] += transA[i][k] * A[k][j];
            }
            for(k = 0; k < 1; ++k)
            {
                multATY[i][j] += transA[i][k] * Y[k][j];
            }
        }
    }

    //cout<<"4444444444444: " << multATA[0][0] <<endl;


    CircleData CD;


    return CD;

}


void kdtreeSearchNearest2D (pcl::PointXY searchPoint, pcl::PointCloud<pcl::PointXY>::Ptr& cloud)
{
	pcl::KdTreeFLANN<pcl::PointXY> kdtree;

	kdtree.setInputCloud (cloud);

	int k = 3;

	vector<int> pointIdxNKNSearch(k);
	vector<float> pointNKNSquaredDistance(k);

	cout << "K nearest neighbor search at (" << searchPoint.x 
		    << " " << searchPoint.y 
		    << ") with K=" << k << std::endl;

	if ( kdtree.nearestKSearch (searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
			std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< " " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

}

bool kdtreeSearchNearest3D (PointT searchPoint,
                            PointCloud::Ptr& cloud,
                            PointT& nearestPoint, double resolution)
{
        pcl::KdTreeFLANN<PointT> kdtree;

	kdtree.setInputCloud (cloud);

	int k = 1;

	vector<int> pointIdxNKNSearch(k);
	vector<float> pointNKNSquaredDistance(k);


	if ( kdtree.nearestKSearch (searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
/*
		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
			std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< " " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< " " << cloud->points[ pointIdxNKNSearch[i] ].z 
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
			cout<<"hahaha: "<< cloud <<endl;
*/
		if (pointNKNSquaredDistance[0] < resolution){
			nearestPoint = cloud->points[ pointIdxNKNSearch[0] ];
			return true;
		}
	}
	return false;
}



CircleData findCircleCenterBounding(PointCloud::Ptr linePCL)
{
    Points points;
    for(int i = 0; i < linePCL->points.size(); i++)
    {
        points.push_back(Point_2(linePCL->points[i].x, linePCL->points[i].y));
    }

//    points.push_back(Point_2(0,0));
//    points.push_back(Point_2(1,1));
//    points.push_back(Point_2(0,1));
//    points.push_back(Point_2(1,0));

//    Min_circle mc1( &points[0], &points[linePCL->points.size() - 1], false);
    Min_circle mc2( &points[0], &points[linePCL->points.size() - 1], true);

    Traits::Circle c = mc2.circle();
    //CGAL::set_pretty_mode( std::cout);
    //std::cout << mc2 <<endl;

    CircleData CD;

    CD.center.x = c.center()[0];
    CD.center.y = c.center()[1];
    CD.radius = sqrt(c.squared_radius());
    CD.height = linePCL->points[0].z;

    return CD;

}



bool isCircleGood(CircleData circle, CircleData centers_cylinder)
{
        if (isClosePoint2D(circle.center.x, circle.center.y,
                           centers_cylinder.center.x, centers_cylinder.center.y, 0.4) &&
                circle.radius > min_radius &&
                circle.radius < max_radius &&
                fabs(circle.radius - centers_cylinder.radius) < 0.5)
	{
		return true;
	}else
	{
		return false;
	}
}

bool isConnected(PointT pre_point, PointT point, float threshold)
{
	if (fabs(pre_point.z - point.z) < threshold && fabs(pre_point.x - point.x) < threshold)
	{
		return true;	 
	}
	return false;
}


void cylinder_detect(PointCloud::ConstPtr cloud)
{
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ExtractIndices<PointT> extract;

    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ())
    std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
      std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
      writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }
}



boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloud::ConstPtr cloud,
                                                                vector < PointCloud::Ptr > slicedCloud3d)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

//    for (int i = 0; i < slicedCloud3d.size(); i++)
//    {
//        //std::cerr << "pointSize: "<< slicedCloud3d[i]->points.size()<< std::endl;
//        stringstream ss;
//        ss << i;
//        viewer->addPointCloud<PointT> (slicedCloud3d[i], ss.str());
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ss.str());
//    }

    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

	
//    for (int n = 0; n < circles_box.size(); n++){
//        for (int m = 0; m < circles_box[m].size(); m++){
			
//            stringstream ss;
//            ss << n << m;
			
//            //cout<<"hehe: "<<circles_box[n][m].center<<" "<<circles_box[n][m].radius<<" "<<circles_box[n][m].height<<endl;
//            viewer->addCylinder (circles_box[n][m].center.x, circles_box[n][m].center.y, circles_box[n][m].height-(slice_step/2),
//                                circles_box[n][m].center.x, circles_box[n][m].center.y, circles_box[n][m].height+(slice_step/2),
//                                circles_box[n][m].radius, 20, ss.str());
//        }
//    }
    //viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");
	
    //viewer->addCylinder<pcl::PointXYZ> (double p0x,double p0y,double p0z,
    //double p1x,double p1y,double p1z, double rad, int numsides, const std::string &id, int viewport);

  return (viewer);
}

int isInRange(double initHeight, double height, double thickness)
{
	if (height < initHeight + (thickness/2) && height >= initHeight)
		return -1;
	else if (height >= initHeight + (thickness/2) && height < initHeight + thickness)
		return 1;
	
	return 0;
}

bool cloud_passthrough (pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_out, double slice_height, double thickness)
{

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

void pointCloudSlice(vector< vector < pcl::PointCloud<PointT>::Ptr > >& sliced_cloud,
                     vector<double>& sliceHeights,
                     pcl::PointCloud<PointT>::Ptr cloud,
                     pcl::ModelCoefficients::Ptr plane_coefficient,
                     double resolution,
                     int sliceConst = 1)
{
    double thickness = sliceConst * resolution;
    double currentHeight = -plane_coefficient->values[3] + 3;


    bool sliceScanned = false;

    while(1)
    {

        pcl::PointCloud<PointT>::Ptr cloudUpperSlice(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloudLowerSlice(new pcl::PointCloud<PointT>);

        //currentHeight += step*thickness;
        //cout<<"eeeeeeeee: "<<currentHeight<<endl;
        if(cloud_passthrough (cloud, cloudUpperSlice, currentHeight, thickness) &&
            cloud_passthrough (cloud, cloudLowerSlice, currentHeight-(thickness), thickness))
        {
            sliceScanned = true;
            vector< pcl::PointCloud<PointT>::Ptr> oneSlice;
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

void findIntersection(pcl::PointXYZ pointa, pcl::PointXYZ pointb, double plane, pcl::PointXYZ& intersection)
{
	double ratea = fabs(plane - pointa.z) / fabs(pointa.z - pointb.z);
	double rateb = fabs(plane - pointb.z) / fabs(pointa.z - pointb.z);

	if (pointa.x > pointb.x)
	{
		intersection.x = pointb.x + (rateb * fabs(pointa.x - pointb.x));
	} else {
		intersection.x = pointa.x + (ratea * fabs(pointa.x - pointb.x));
	}

	if (pointa.y > pointb.y)
	{
		intersection.y = pointb.y + (rateb * fabs(pointa.y - pointb.y));
	} else {
		intersection.y = pointa.y + (ratea * fabs(pointa.y - pointb.y));
	}
	intersection.z = plane;

}



void form2DSlices(vector< vector < PointCloud::Ptr > > sliced_cloud,
                  vector < pcl::PointCloud<PointT>::Ptr >& clouds, vector<double> sliceHeights, double resolution)
{	

	for (int i = 0; i < sliced_cloud.size(); i++)
	{


		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//        for (int j = 0; j < sliced_cloud[i][0]->points.size(); j++)
//		{
//            pcl::PointXYZ intersectionPoint;
//            pcl::PointXYZ nearestPoint;
//            //find intersection of two points with slice plane
//            if (kdtreeSearchNearest3D (sliced_cloud[i][0]->points[j], sliced_cloud[i][1], nearestPoint, resolution)){
//                findIntersection(sliced_cloud[i][0]->points[j], nearestPoint, sliceHeights[i], intersectionPoint);
//                cloud->points.push_back(intersectionPoint);

////                cout<<"heheda111111: "<<sliced_cloud[i][0]->points[j]<<endl;
////                cout<<"heheda222222: "<<nearestPoint<<endl;
////                cout<<"intersectionPoint: "<<intersectionPoint<<endl;
//            }
//		}
        //cv::Mat image(rows, cols, CV_8U, 255);

        //bubbleSort(cloud, cloud->points.size());

        for (int j = 0; j < sliced_cloud[i][0]->points.size(); j++)
        {
            PointT intersectionPoint;
            intersectionPoint.x = sliced_cloud[i][0]->points[j].x;
            intersectionPoint.y = sliced_cloud[i][0]->points[j].y;
            intersectionPoint.z = sliceHeights[i];
            cloud->points.push_back(intersectionPoint);
        }
        for (int j = 0; j < sliced_cloud[i][1]->points.size(); j++)
        {
            PointT intersectionPoint;
            intersectionPoint.x = sliced_cloud[i][1]->points[j].x;
            intersectionPoint.y = sliced_cloud[i][1]->points[j].y;
            intersectionPoint.z = sliceHeights[i];
            cloud->points.push_back(intersectionPoint);
        }


		clouds.push_back(cloud);
	}
}



bool isIncludePoint (pcl::PointCloud<PointT>::Ptr cloud, PointT point, vector<int> pointIdxNKNSearch)
{
	for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	{
		//cout<<"heheda111111: "<<cloud->points[ pointIdxNKNSearch[i] ]<<endl;
		//cout<<"heheda222222: "<<point<<endl;
		if (cloud->points[ pointIdxNKNSearch[i] ].x == point.x
			&& cloud->points[ pointIdxNKNSearch[i] ].y == point.y
			&& cloud->points[ pointIdxNKNSearch[i] ].z == point.z)
		{
			return true;
		}	
	}
	return false;
}



void orderPoints(pcl::PointCloud<PointT>::Ptr cloud, vector< pcl::PointCloud<PointT>::Ptr >& segmentedCloud2d,
                 vector< CircleData >& circles, double resolution)
{
	PointT prePoint = cloud->points[0];
	pcl::PointCloud<PointT>::Ptr tempCloud(new pcl::PointCloud<PointT>);
	int index = 0;
	int num_points = cloud->points.size();

	for (int i = 0; i < num_points; i++)
	{
            if (i == 0){
                    segmentedCloud2d.push_back(tempCloud);
                    segmentedCloud2d[0]->points.push_back(cloud->points[i]);
                    cloud->erase(cloud->begin());
                    continue;
            }

            pcl::KdTreeFLANN<PointT> kdtree;
            kdtree.setInputCloud (cloud);

            int k = 2;

            vector<int> pointIdxNKNSearch(k);
            vector<float> pointNKNSquaredDistance(k);

            if ( kdtree.nearestKSearch (segmentedCloud2d[index]->points.back(), k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                //cout<<"true: "<< pointNKNSquaredDistance[0] <<endl;
                if (pointNKNSquaredDistance[0] < (resolution * 20))
                {

                        segmentedCloud2d[index]->points.push_back(cloud->points[pointIdxNKNSearch[0]]);

                } else {
                if (tempCloud->points.size() > 5)
                    {
                            CircleData circle = findCircleCenterLeast(tempCloud);
                            circles.push_back(circle);
                            //cout<<"circle center: "<<circle.center<<"circle radius: "
                            //<<circle.radius<<"circle height: "<<circle.height<<endl;
                    }

                    tempCloud->points.clear();
                    segmentedCloud2d.push_back(tempCloud);
                    index++;
                    segmentedCloud2d[index]->points.push_back(cloud->points[pointIdxNKNSearch[0]]);

                }
                cloud->erase(cloud->begin()+pointIdxNKNSearch[0]);
            }
	}
	if (index == 0)
	{
		CircleData circle;
		circles.push_back(circle);
	}
}



void segmentSlicedPoints(vector < pcl::PointCloud<PointT>::Ptr > sliced_cloud, vector< vector< CircleData > > circles_box, double resolution)
{

	int size = sliced_cloud.size();
	for (int i = 0; i < size; i++)
    {
		pcl::PointCloud<PointT>::Ptr cloud = sliced_cloud[i];
		vector< CircleData > circles;
        pcl::PointCloud<PointT>::Ptr segmentedCloud2d;
        vector<PointT> tempClouds;
        //orderPoints(sliced_cloud[i], segmentedCloud2d, circles, resolution);
        for (int j = 0; j < cloud->points.size()-1; j++)
        {
            if (isConnected(cloud->points[j], cloud->points[j+1], resolution*4))
            {
                tempClouds.push_back(cloud->points[0]);
            }else{
                //CircleData circle = findCircleCenter(segmentedCloud2d);
                cout<<"jjee__"<< i <<" : "<<tempClouds.size()<<endl;
                tempClouds.clear();
            }
        }
        //circles_box.push_back(circles);
		
	}
}


void downsample(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<PointT>::Ptr& cloud_output)
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

void outlierFilte(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<PointT>::Ptr& cloud_output)
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

void planeDetect(pcl::PointCloud<PointT>::Ptr cloud_input,
                 pcl::ModelCoefficients::Ptr& plane_coefficient)
{
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.1);


    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_input);
    seg.segment (*inliers, *plane_coefficient);



}

void translateHorizental(pcl::PointCloud<PointT>::Ptr cloud_input,
                         pcl::PointCloud<PointT>::Ptr& cloud_output,
                         pcl::ModelCoefficients::Ptr& plane_coefficient)
{


    //std::cerr << "Plane coefficients: " << *plane_coefficient << std::endl;

    /**  Using a Affine3f
      This method is easier and less error prone
    */

    float thetax = atan((plane_coefficient->values[1])/plane_coefficient->values[2]);
    Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
    // The same rotation matrix as before; theta radians arround X axis
    transform_x.rotate (Eigen::AngleAxisf (thetax, Eigen::Vector3f::UnitX()));
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud_input, *cloud_output, transform_x);

//    float thetay = atan(-(plane_coefficient->values[0])/plane_coefficient->values[2]);
//    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
//    // The same rotation matrix as before; theta radians arround Y axis
//    transform_y.rotate (Eigen::AngleAxisf (thetay, Eigen::Vector3f::UnitY()));
//    // You can either apply transform_1 or transform_2; they are the same
//    pcl::transformPointCloud (*cloud_output, *cloud_output, transform_y);

//    float thetaz = atan(-(plane_coefficient->values[0])/plane_coefficient->values[1]);
//    Eigen::Affine3f transform_z = Eigen::Affine3f::Identity();
//    // The same rotation matrix as before; theta radians arround Z axis
//    transform_z.rotate (Eigen::AngleAxisf (thetaz, Eigen::Vector3f::UnitZ()));
//    // You can either apply transform_1 or transform_2; they are the same
//    pcl::transformPointCloud (*cloud_output, *cloud_output, transform_z);

}

void clustering2DPoints(pcl::PointCloud<PointT>::Ptr cloud_input, vector<pcl::PointCloud<PointT>::Ptr>& clusters)
{
    /**
    * clustering
    */
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_input);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.08); // 10cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_input);
    ec.extract (cluster_indices);


    int j = 0;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {

          cloud_cluster->points.push_back (cloud_input->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        //std::stringstream ss;
        //ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<PointT> (ss.str (), *cloud_cluster, false);

        if (!cloud_cluster->points.empty())
            clusters.push_back(cloud_cluster);
        j++;
    }
}




void
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    pcl::ModelCoefficients cylinder_coeff;
    //cylinder_coeff.values.resize (7);    // We need 7 values
    cylinder_coeff.values.push_back( 0.0 );
    cylinder_coeff.values.push_back( 0.0 );
    cylinder_coeff.values.push_back( 0.0 );
    cylinder_coeff.values.push_back( 0.0 );
    cylinder_coeff.values.push_back( 0.0 );
    cylinder_coeff.values.push_back( 1.0 );
    cylinder_coeff.values.push_back( 1.0 );

    viewer.removeShape ("cylinder", 0);
    viewer.addCylinder (cylinder_coeff.values[0],
                        cylinder_coeff.values[1],
                        cylinder_coeff.values[2],
                        cylinder_coeff.values[3],
                        cylinder_coeff.values[4],
                        cylinder_coeff.values[5],
                        cylinder_coeff.values[6], 20, "cylinder", 0);


}

void *PrintHello(void *threadid) {
   long tid;
   tid = (long)threadid;
   cout << "Hello World! Thread ID, " << tid << endl;
   pthread_exit(NULL);
}

bool optimizeCylinder(vector<CircleData> cylinder_input, pcl::ModelCoefficients::Ptr& cylinder_output)
{

    double x = cylinder_input[0].center.x;
    double y = cylinder_input[0].center.y;
    double radius = cylinder_input[0].radius;
    double errors = fabs(radius - cylinder_input[1].radius);

    int gap = 0;

    for (int i = 1; i < cylinder_input.size(); i++)
    {

        if (fabs(radius - cylinder_input[i].radius) < 1.3*errors &&
                fabs(cylinder_input[i].radius - cylinder_input[i+1].radius) < 1.3*errors)
        {
            x = (x + cylinder_input[i].center.x)/2;
            y = (y + cylinder_input[i].center.y)/2;

            radius = (radius + cylinder_input[i].radius)/2;
            //errors = (errors + fabs(radius - cylinder_input[i].radius))/2;
        }

        if (fabs(cylinder_input[i].height - cylinder_input[i-1].height) > 0.45)
        {
            gap+=3;

        }else if (fabs(cylinder_input[i].height - cylinder_input[i-1].height) > 0.3)
        {
            gap+=2;

        }else if (fabs(cylinder_input[i].height - cylinder_input[i-1].height) > 0.15)
        {
            gap+=1;
        }
    }


    if (gap < 3)
    {
        cylinder_output->values.push_back( x );
        cylinder_output->values.push_back( y );
        cylinder_output->values.push_back( cylinder_input[0].height + (slice_step * step)/2 );
        cylinder_output->values.push_back( x );
        cylinder_output->values.push_back( y );
        cylinder_output->values.push_back( cylinder_input[cylinder_input.size()-1].height - (slice_step * step)/2);
        cylinder_output->values.push_back( radius );
        return true;
    }

    return false;

//    cout<<"height dadada "<<": "<< cylinder_input[0].height - (slice_step * step)/2 <<endl;
//    cout<<"height xiao "<<": "<< cylinder_input[cylinder_input.size()-1].height + (slice_step * step)/2 <<endl;

}

bool isTooClosedCircles(CircleData circle_1, CircleData circle_2)
{
    float distance = sqrt((fabs(circle_1.center.x - circle_2.center.x)*fabs(circle_1.center.x - circle_2.center.x))
                          + (fabs(circle_1.center.y - circle_2.center.y)*fabs(circle_1.center.y - circle_2.center.y)));
    if ( distance < (circle_1.radius + circle_2.radius)*0.8 )
    {
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief findCylinder
 * @param centers
 * @param cylinders
 *
 * find cylinder from slice circles
 */
void findCylinder(vector<CircleData> circles, vector< pcl::ModelCoefficients::Ptr>& cylinders)
{

    //cout<< "circles: "<<circles.size()<<endl;

    vector< vector<CircleData> > centers_cylinder;

    pcl::PointCloud<PointT>::Ptr temp_centers (new pcl::PointCloud<PointT>());

    for (int i = 0; i < circles.size(); i++)
    {

        //temp_centers->points.push_back(center2D);

        bool haveClosePoint = false;

        if (i > 0)
        {
            for (int j = 0; j < centers_cylinder.size(); j++)
            {
                for (int k = 0; k < centers_cylinder[j].size(); k++)
                {
                    if(isCircleGood(circles[i], centers_cylinder[j][k]))
                    {
                        haveClosePoint = true;
                        centers_cylinder[j].push_back(circles[i]);
                        break;
                    }
                }
            }
        }

        //cout<<"iiiiiii: "<< i<<" "<<haveClosePoint<<endl;

        if (!haveClosePoint)
        {
            vector<CircleData> cylinder_hypo;
            cylinder_hypo.push_back(circles[i]);
            centers_cylinder.push_back(cylinder_hypo);
        }

    }



    for (int i = 0; i < centers_cylinder.size(); i++)
    {
        bool hasCircleInside = false;

        if (centers_cylinder[i].size() > 3)
        {

            for (int j = 0; j < centers_cylinder.size(); j++)
            {
                if (i != j)
                {
                    if (isTooClosedCircles(centers_cylinder[i][0], centers_cylinder[j][0]))
                    {
                        hasCircleInside = true;
                        break;
                    }
                }
            }

            if (!hasCircleInside)
            {
                pcl::ModelCoefficients::Ptr cylinder (new pcl::ModelCoefficients());

                if (optimizeCylinder(centers_cylinder[i], cylinder))
                {
                    //            cylinder->values[0] = centers_cylinder[i][0].x;
                    //cout<<"hehe: "<<i<<endl;
                    cylinders.push_back(cylinder);
                }
            }
        }
    }

//    cout<< "cylinders: "<<cylinders.size()<<endl;
}

int reginalStatusDef(pcl::PointCloud<PointT>::Ptr slicedCloud, int currentLvl, int totalvl)
{
    if (currentLvl < (totalvl/5))
    {
        return atGround;
    }else if (currentLvl < (3*totalvl/5))
    {
        return atTrunk;
    }else
    {
        return atCrown;
    }
}

void groundRemove()
{

}

void convexHull(pcl::PointCloud<PointT>::Ptr cluster, pcl::PointCloud<PointT>::Ptr& cluster_hull)
{
    //2D Convex Hulls and Extreme Points
//    cout<<"slice_clusters: "<<cluster->points.size()<<endl;

    Points points, result;
    for(int i = 0; i < cluster->points.size(); i++)
    {
        points.push_back(Point_2(cluster->points[i].x, cluster->points[i].y));
    }

    CGAL::convex_hull_2( points.begin(), points.end(), std::back_inserter(result) );
//    cout << result.size() << " points on the convex hull" << std::endl;

    for(int i = 0; i < result.size(); i++)
    {

        PointT cloudPoint;
        cloudPoint.x = result[i][0];
        cloudPoint.y = result[i][1];
        cloudPoint.z = cluster->points[0].z;

        cluster_hull->points.push_back(cloudPoint);
//        cout<<result[i][0]<<endl;
    }
}

/**
 * @brief trendDefine
 * @param preSize
 * @param curSize
 * @param nextSize
 * @return -1 = decrease. 0 = unchange. 1 = increase
 */
int trendDefine(int& preSizeAvg, int curSize, int nextSize){

    cout<<"11111111111112: "<<endl;

    if((preSizeAvg - curSize) > preSizeAvg/10 && (curSize - nextSize) > curSize/10)
    {
        preSizeAvg = (preSizeAvg + (curSize * 2) + (nextSize * 4))/6;
        return -1;
    }
    else if ((curSize - preSizeAvg) > preSizeAvg/10 && (nextSize - curSize) > preSizeAvg/10)
    {
        preSizeAvg = (preSizeAvg + (curSize * 2) + (nextSize * 4))/6;
        return 1;
    }
    else
    {
        preSizeAvg = (preSizeAvg + (curSize * 2) + (nextSize * 4))/6;
        return 0;
    }
}


void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    vector<CircleData> circles;

    pthread_t threads;


    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish (output);

    pcl::PointCloud<PointT>::Ptr cloud_rgb (new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*cloud, *cloud_rgb);

    pthread_create(&threads, NULL, PrintHello, (void *)cloud_rgb->points.size());
    //point cloud resolution
    //double resolution = computeCloudResolution(cloud_rgb, 100);
    //cout<<"resolution: "<< resolution<<endl;


    //viewer.showCloud(cloud_rgb);

    //viewer.runOnVisualizationThread (viewerOneOff);

}

void
depth_cb (const sensor_msgs::ImageConstPtr& depth_msg)
{

    cv_bridge::CvImagePtr img_ptr_rgb;
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

    cv::Mat& mat_depth = img_ptr_depth->image;

//    cv::Mat image;
//    image = imread(*mat_depth, cv::CV_LOAD_IMAGE_COLOR);   // Read the file

//    if(! image.data )                              // Check for invalid input
//    {
//        cout <<  "Could not open or find the image" << std::endl ;

//    }

//    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//    cv::imshow( "Display window", image );

//    cv::waitKey(0);
}




int main (int argc, char** argv)
{
    if (camera == true){
        // Initialize ROS
        ros::init (argc, argv, "my_pcl_tutorial");
        ros::NodeHandle nh;

        // Create a ROS subscriber for the input point cloud
        //ros::Subscriber sub = nh.subscribe ("/voxel_cloud", 1000, cloud_cb);
        ros::Subscriber sub = nh.subscribe ("/camera/depth/image_raw", 1000, depth_cb);

        // Create a ROS publisher for the output point cloud
        pub = nh.advertise<sensor_msgs::PointCloud2> ("outputheheda", 1);

        //my_viewer = simpleVis(cloud_empty);

        // Spin
        ros::spin ();
    }else{


        vector< vector< CircleData > > circles_box;
        pcl::PCDReader reader;
        pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_outlier_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr tmpPoints (new pcl::PointCloud<PointT>());
        vector<double> sliceHeights;

        /**
         * Load Clouds
         */
        string scene_filename_ = argv[1];

        if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
        {
          std::cout << "Error loading scene cloud." << std::endl;

          return (-1);
        }

        vector< pcl::PointCloud<PointT>::Ptr > cloud_outputs;
        pcl::PointCloud<PointT>::Ptr cloud_view = scene;

        /**
        * use downsampling filter
        */

        //voxelFilte(scene, cloud_voxel_filtered);
        //downsample(scene, cloud_voxel_filtered);

        /**
        * outlier filter
        */
        //outlierFilte(cloud_voxel_filtered, cloud_outlier_filtered);

        /**
        * translate cloud horizentalize with detected floor plane
        */
        pcl::ModelCoefficients::Ptr plane_coefficient (new pcl::ModelCoefficients);
        pcl::PointCloud<PointT>::Ptr cloud_translated (new pcl::PointCloud<PointT>);
        planeDetect(scene, plane_coefficient);

        translateHorizental(scene, cloud_translated, plane_coefficient);

        //writer.write ("cloud2.pcd", *cloud_translated, false);

        /**
        * translate cloud horizentalize with detected floor plane
        */
        double resolution = computeCloudResolution(cloud_translated, 100);
        cout<<"resolution: "<< resolution<<endl;
//        cylinder_detect(cloud_view);

        //point cloud slice
        vector<vector < pcl::PointCloud<PointT>::Ptr > > slicedCloud3d;
        vector < pcl::PointCloud<PointT>::Ptr > slicedCloud2d;

        pointCloudSlice(slicedCloud3d, sliceHeights, cloud_translated, plane_coefficient, resolution, step);

        //from 3D to 2D
        form2DSlices(slicedCloud3d, slicedCloud2d, sliceHeights, resolution);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

        viewer = simpleVis(cloud_translated, slicedCloud2d);
        //viewer->addPlane(*plane_coefficient, "plane");
        //segmented in 2D
        vector<CircleData> circles;

        int previousCloudAvgSize = slicedCloud2d[0]->points.size();

        for (int i = 0; i < slicedCloud2d.size(); i++)
        {

            

//            cout<<"slice_clusters: "<<slicedCloud2d[i]->points.size()<<endl;
//            if(slicedCloud2d[i]->points.size() > 5000)
//            {
//            }
                //continue;

//            if (reginalStatusDef(slicedCloud2d[i], i, slicedCloud2d.size()) >= 0)
//            {

            //reginalStatusDef(slicedCloud2d[i], i, slicedCloud2d.size());

            //cout<<"slicedCloud2d: "<<currentStatus<<" -- "<<slicedCloud2d[i]->points.size()<<endl;
            

//            if(i > 0 && i < slicedCloud2d[i]->points.size() && segmentForest)
//            {
            
//                if (currentStatus == atGround)
//                {
//                    if(trendDefine(previousCloudAvgSize, slicedCloud2d[i]->points.size(), slicedCloud2d[i+1]->points.size()) == -1)
//                    {
//                        currentStatus = atTrunkAssume;
//                    }
//                } else if (currentStatus == atTrunkAssume)
//                {
//                    if (trendDefine(previousCloudAvgSize, slicedCloud2d[i]->points.size(), slicedCloud2d[i+1]->points.size()) == 0)
//                    {
//                        currentStatus = atTrunk;
//                    }
//                } else if (currentStatus == atTrunk)
//                {
//                    if (trendDefine(previousCloudAvgSize, slicedCloud2d[i]->points.size(), slicedCloud2d[i+1]->points.size()) == 1)
//                    {
//                        currentStatus = atCrown;
//                    }
//                } else if (currentStatus == atCrownAssume)
//                {

//                } else
//                {
//                }
//            }
            
//            stringstream ss;
//            ss << i ;

            vector<pcl::PointCloud<PointT>::Ptr> slice_clusters;

            clustering2DPoints(slicedCloud2d[i], slice_clusters);
//            cout<<"slicedCloud2d size: "<<slicedCloud2d[i]->points.size()<<endl;
//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(slicedCloud2d[i],
//                                                                                         rand() % 255, rand() % 255, rand() % 255);
//            viewer->addPointCloud<PointT> (slicedCloud2d[i], single_color, ss.str());
            for (int j = 0; j < slice_clusters.size(); j++)
            {

                //cout<<"slice_clusters: "<<slice_clusters[j]->points.size()<<endl;

                //2D Convex Hulls and Extreme Points
                pcl::PointCloud<PointT>::Ptr cluster_hull (new (pcl::PointCloud<PointT>));
                convexHull(slice_clusters[j], cluster_hull);

                stringstream ss;
                ss << i << j;
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cluster_hull,
//                                                                                             rand() % 255, rand() % 255, rand() % 255);

//                viewer->addPointCloud<PointT> (cluster_hull, single_color, ss.str());

//                if (currentStatus == atGround || currentStatus == atTrunkAssume)
//                {
//                    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(slice_clusters[j],
//                                                                                                 255, 0, 0);
//                    viewer->addPointCloud<PointT> (slice_clusters[j], single_color, ss.str());
//                }else if (currentStatus == atTrunk )
//                {
//                    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cluster_hull,
//                                                                                                 0, 255, 0);
//                    viewer->addPointCloud<PointT> (cluster_hull, single_color, ss.str());
//                }else
//                {
//                    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cluster_hull,
//                                                                                                 0, 0, 255);
//                    viewer->addPointCloud<PointT> (cluster_hull, single_color, ss.str());
//                }

//                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ss.str());

                /**
                 * @brief circleData find circle center method one
                 *
                 */

                //CircleData circleData = findCircleCenter(slice_clusters[j]);

                //findCircleCenterLeast

                /**
                 * @brief circleData find circle center method two
                 *
                 */

                CircleData circleData;
                if (1)
                {

                    
                    circleData = findCircleCenterBounding(cluster_hull);

                    //if (isCircleGood(circleData))
                    //{
                        circles.push_back(circleData);
                    //}
                }


                /**
                 * @brief circleData find circle center method three
                 *
                 */

                //CircleData circleData = findCircleCenterBounding(cluster_hull);

                ss << "circle" << i <<j;
                
//                cout<<"circleData center: " << circleData.center <<endl;
//                cout<<"circleData height: " << circleData.height <<endl;
//                cout<<"circleData radius: " << circleData.radius <<endl;

                if (circleData.radius < max_radius)
                {


//                    viewer->addCube(circleData.center.x - (circleData.radius),
//                                    circleData.center.x + (circleData.radius),
//                                    circleData.center.y - (circleData.radius),
//                                    circleData.center.y + (circleData.radius),
//                                    circleData.height - 0.05,
//                                    circleData.height + 0.05,
//                                    100.0, 100.0, 100.0, ss.str());

//                        viewer->addCylinder(circleData.center.x, circleData.center.y, circleData.height-(slice_step/8),
//                        circleData.center.x, circleData.center.y, circleData.height+(slice_step/8), circleData.radius, 20, ss.str());
//                        PointT tmpPoint;
//                        tmpPoint.x = circleData.center.x;
//                        tmpPoint.y = circleData.center.y;
//                        tmpPoint.z = circleData.height;
//                        tmpPoints->points.push_back(tmpPoint);

                }
//                }
            }
        }

        //cout<<"222222222222"<<endl;
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(tmpPoints, 0, 255, 0);
//        viewer->addPointCloud<PointT> (tmpPoints, single_color, "hehe");
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "hehe");


        vector< pcl::ModelCoefficients::Ptr> cylinders;

        findCylinder(circles, cylinders);
        cout<< "cylinders_size: "<<cylinders.size()<<endl;



        if (cylinders.size() > 0)
        {
            for (int i = 0; i < cylinders.size(); i++)
            {
                cout<< "tree_ "<< i <<" _: \nx: "<< cylinders[i]->values[0] <<
                       " \ny: " << cylinders[i]->values[1] <<
                       "\nTree Height (visiable area): " << fabs(cylinders[i]->values[2] - cylinders[i]->values[5])<<
                       "\nTree radius: "<< cylinders[i]->values[6] <<endl;
                stringstream ss;
                ss << i;
//                cout<<"hehe: "<<cylinders[i]->values[0] - (cylinders[i]->values[6] / 2)<<
//                    cylinders[i]->values[0] + (cylinders[i]->values[6] / 2)<<
//                    cylinders[i]->values[1] - (cylinders[i]->values[6] / 2)<<
//                    cylinders[i]->values[1] + (cylinders[i]->values[6] / 2)<<
//                    cylinders[i]->values[2]<<
//                    cylinders[i]->values[5]<<endl;
//                viewer->addCylinder(0,0,0,0,0,1,3, 20, ss.str());
//                pcl::ModelCoefficients::Ptr cube_coefficients (new pcl::ModelCoefficients);
//                cubeTmp.resize(4);
//                cube_coefficients->values[0] = ;
//                viewer->addCylinder(cylinders[i]->values[0],
//                                    cylinders[i]->values[1],
//                                    cylinders[i]->values[2],
//                                    cylinders[i]->values[3],
//                                    cylinders[i]->values[4],
//                                    cylinders[i]->values[5],
//                                    cylinders[i]->values[6], 20, ss.str());


                //cout<<"dada xiaoxiao: "<<cylinders[i]->values[2]<<"  "<< cylinders[i]->values[5]<<endl;

                viewer->addCube(cylinders[i]->values[0] - (cylinders[i]->values[6]),
                                cylinders[i]->values[0] + (cylinders[i]->values[6]),
                                cylinders[i]->values[1] - (cylinders[i]->values[6]),
                                cylinders[i]->values[1] + (cylinders[i]->values[6]),
                                cylinders[i]->values[5],
                                cylinders[i]->values[2],
                                100.0, 100.0, 100.0, ss.str());

//                    viewer->addCube(0.1, 2, 0.1, 2, 0.1, 2,
//                                    1.0, 1.0, 1.0, ss.str());
            }

        }

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

        //cv::waitKey(0);

    }

}




