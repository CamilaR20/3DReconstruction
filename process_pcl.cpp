#include <iostream>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/poisson.h>

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

int main(int argc, char* argv[]) {
    // Cargar nube de puntos
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(argv[1], *cloud);

    // Información sobre la nube de puntos
    cout << "Cloud points: " << cloud->points.size() << endl;

    // Mostrar nube de puntos
//    pcl::visualization::PCLVisualizer::Ptr viewer;
//    viewer = simpleVis(cloud);
//    while (!viewer->wasStopped ()){
//        viewer->spinOnce (100);
//        std::this_thread::sleep_for(100ms);
//    }

    // Filtrar nube de puntos
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // Crear objeto para filtrar
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);    //obj 100 - 1 atof(argv[2])
    sor.setStddevMulThresh (1);
    sor.filter (*cloud_filtered);

    cout << "Cloud points after statistical outlier removal: " << cloud_filtered->points.size() << endl;

//    viewer = simpleVis(cloud_filtered);
//    while (!viewer->wasStopped ()){
//        viewer->spinOnce (100);
//        std::this_thread::sleep_for(100ms);
//    }

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    // two k- dimensional trees which are used to organize the cloud points in k dimensions
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2;

    // Create search tree...
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
    tree->setInputCloud(cloud_filtered);

//    // Normal estimation...
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
//    n.setInputCloud(cloud_filtered);
//    n.setSearchMethod(tree);
//    n.setKSearch(20);
//    n.compute(*normals);
//
//    //Concatenate XYZ and normal information..
//    pcl::concatenateFields (*cloud_filtered,*normals,*cloud_normals);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);
    // Set parameters
    mls.setInputCloud (cloud_filtered);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);
    mls.process (*cloud_normals);

    cout << "Cloud points after MLS smoothing: " << cloud_normals->points.size() << endl;

    // Reconstruction
    // The PolygonMesh data type contains information about vertex position, edge data and face data.
    //This data is required for PCL to save the new reconstructed mesh as a file that can be read by another 3D software.
    //boost::shared_ptr<pcl::PolygonMesh> mesh (new pcl::PolygonMesh);
    pcl::PolygonMesh mesh;

    // Poisson
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth (10);
    poisson.setInputCloud (cloud_normals);
    poisson.reconstruct(mesh);

    // Laplacian Smoothing
//    pcl::PolygonMesh output;
//    pcl::PolygonMesh::ConstPtr input (&mesh);
//    const pcl::PolygonMesh *input = &mesh;
//    pcl::MeshSmoothingLaplacianVTK vtk;
//    vtk.setInputMesh(pcl::PolygonMesh::ConstPtr(input));
//    vtk.setNumIter (20000);
//    vtk.setConvergence (0.0001);
//    vtk.setRelaxationFactor (0.0001);
//    vtk.setFeatureEdgeSmoothing (true);
//    vtk.setFeatureAngle (M_PI/5);
//    vtk.setBoundarySmoothing (true);
//    vtk.process(output);

    string filePath = "/Users/camilaroa/CLionProjects/PointClouds/final1.ply";
    pcl::io::savePLYFileBinary(filePath, mesh);

    return 0;
}