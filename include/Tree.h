
#ifndef TREE_H
#define TREE_H

#include <ros/ros.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;


class Tree
{
public:
    Tree();
    ~Tree();


private:

    PointCloud::Ptr trunck;
    PointCloud::Ptr crown;

};







#endif
