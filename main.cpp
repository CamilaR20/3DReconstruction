#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
using namespace std;

int main(int argc, char* argv[]) {

    // Load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(argv[1], *cloud);

    // Point cloud info
    cout << "Cloud points: " << cloud->points.size() << endl;

    // Filter point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (atof(argv[2]));    //obj 50 - 1 atof(argv[2])
    sor.setStddevMulThresh (atof(argv[3]));
    sor.filter (*cloud_filtered);

    cout << "Cloud points after statistical outlier removal: " << cloud_filtered->points.size() << endl;

    // Compute normals of the point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    // k- dimensional tree used to organize the cloud points in k dimensions
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>(false));
    tree->setInputCloud(cloud_filtered);

    // Normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
    n.setInputCloud(cloud_filtered);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //Concatenate XYZ and normal information..
    pcl::concatenateFields (*cloud_filtered,*normals,*cloud_normals);
    cout << "Normal estimation done "<< endl;

    string pathCloud = "/Users/camilaroa/CLionProjects/PointClouds/cloud.ply";
    pcl::io::savePLYFileBinary(pathCloud, *cloud_normals);

    return 0;
}

