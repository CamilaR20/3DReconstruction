// /Users/camilaroa/CLionProjects/PointClouds/obj1.ply /Users/camilaroa/CLionProjects/PointClouds/obj1 n n p
#include <iostream>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>

using namespace std;
using namespace std::chrono_literals;

static void help() {
    cout
            << "\n------------------------------------------------------------------------------------\n"
            << " This program loads a point cloud from a .ply file, filters it with an Statistical\n"
            << " Outlier filter, then it computes the normals and uses a reconstruction method to \n"
            << " create a surface using PCL. \n"
            << " Usage:\n"
            << "    process_pcl <cloud_path> <mesh_path> <show> <normals_method> <reconstruction_method>\n"
            << "------------------------------------------------------------------------------------\n\n"
            << endl;
}

// To visualize Point Cloud
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
    // Check if all input parameters are set
    if ( argc != 6 ){
        help();
        exit(0);
    }

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
    sor.setMeanK (50);    //obj 100 - 1 atof(argv[2])
    sor.setStddevMulThresh (1);
    sor.filter (*cloud_filtered);

    cout << "Cloud points after statistical outlier removal: " << cloud_filtered->points.size() << endl;

    if(string(argv[3])=="y"){
        // Show point cloud
        pcl::visualization::PCLVisualizer::Ptr viewer;
        viewer = simpleVis(cloud);
        while (!viewer->wasStopped ()){
            viewer->spinOnce (100);
            std::this_thread::sleep_for(100ms);
        }
        // Show filtered point cloud
        viewer = simpleVis(cloud_filtered);
        while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
        }
    }

    // Compute normals of the point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    // k- dimensional tree used to organize the cloud points in k dimensions
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>(false));
    tree->setInputCloud(cloud_filtered);

    // Choose normal estimation method
    if(string(argv[4])=="n"){
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
    } else if(string(argv[4])=="mls"){
        // Smoothing and normal estimation based on polynomial reconstruction
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        mls.setComputeNormals (true);
        mls.setInputCloud (cloud_filtered);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.03);
        mls.process (*cloud_normals);
        cout << "Smoothing and normal estimation based on polynomial reconstruction done"<< endl;
        cout << "Cloud points after MLS smoothing: " << cloud_normals->points.size() << endl;
    }

    // Reconstruction
    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_normals);

    // Choose reconstruction method
    if (string(argv[5])=="p"){
        // Poisson
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth (10);
        poisson.setInputCloud (cloud_normals);
        poisson.reconstruct(*mesh);
    } else if (string(argv[5])=="gt"){
        // Greedy Triangulation
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gt;
        gt.setInputCloud (cloud_normals);
        gt.setSearchMethod (tree2);
        gt.setSearchRadius(0.025); // 0.25
        gt.setMu(2.5);
        gt.setMaximumNearestNeighbors(100); // 1000
        gt.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gt.setMinimumAngle(0); // 0 degrees
        gt.setMaximumAngle(M_PI); // 180 degrees
        gt.setNormalConsistency(true);
        gt.setConsistentVertexOrdering(true);
        gt.reconstruct(*mesh);
    } else if (string(argv[5])=="gp"){
        // Grid projection
        pcl::GridProjection<pcl::PointNormal> gp;
        gp.setInputCloud (cloud_normals);
        gp.setSearchMethod(tree2);
        gp.setResolution(0.0025); // 0.01
        gp.setPaddingSize(3); // 4
        gp.reconstruct (*mesh);
    }
    cout << "Reconstruction done "<< endl;

    // Laplacian Smoothing
    pcl::PolygonMesh::Ptr output(new pcl::PolygonMesh);
    pcl::MeshSmoothingLaplacianVTK vtk;
    vtk.setInputMesh(mesh);
    vtk.setNumIter (20000);
    vtk.setConvergence (0.0001);
    vtk.setRelaxationFactor (0.0001);
    vtk.setFeatureEdgeSmoothing (true);
    vtk.setFeatureAngle (M_PI/5);
    vtk.setBoundarySmoothing (true);
    vtk.process(*output);

    // Save mesh
    string pathMesh = string(argv[2]) + "/mesh.ply";
    pcl::io::savePLYFileBinary(pathMesh, *output);

    //string pathCloud = "/Users/camilaroa/CLionProjects/PointClouds/obj1/cloud.ply";
    //pcl::io::savePLYFileBinary(pathCloud, *cloud_normals);

    return 0;
}