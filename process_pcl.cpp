#include <iostream>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace std::chrono_literals;

// Función para visualizar nube de puntos
pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    // -----Open 3D viewer and add point cloud-----
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "Nube de puntos");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Nube de puntos");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    string filePath = "/Users/camilaroa/CLionProjects/PointClouds/obj.ply";
    pcl::io::loadPLYFile(filePath, *cloud);

    // Información sobre la nube de puntos
    cout << "----------------------------" << endl;
    cout << "Número de puntos: " << cloud->points.size() << endl;
    cout << "----------------------------" << endl;

    // Mostrar nube de puntos
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = simpleVis(cloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    // Filtrar nube de puntos
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);    //obj 100 - 1
    sor.setStddevMulThresh (0.5);
    sor.filter (*cloud_filtered);

    cout << "----------------------------" << endl;
    cout << "Statistical outlier removal: " << endl;
    cout << "============================" << endl;
    cout << "Cloud points: " << cloud_filtered->points.size() << endl;
    cout << "----------------------------" << endl;

    viewer = simpleVis(cloud_filtered);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    //string filePath = "/Users/camilaroa/CLionProjects/PointClouds/obj2.ply";
    //pcl::io::savePLYFileBinary(filePath, *cloud_filtered);

    // https://pcl.readthedocs.io/en/latest/hull_2d.html#hull-2d


    return 0;
}