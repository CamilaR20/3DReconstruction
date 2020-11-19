// /Users/camilaroa/CLionProjects/imgs/obj/obj_paths.txt /Users/camilaroa/CLionProjects/imgs/calibration.json /Users/camilaroa/CLionProjects/PointClouds

#define CERES_FOUND true    // https://answers.opencv.org/question/201467/problem-with-some-functions-of-sfm-module/
#include <iostream>
#include <thread>

#include <nlohmann/json.hpp>

#include <opencv2/sfm.hpp>
#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace std::chrono_literals;
using namespace cv;
using namespace cv::sfm;
using json = nlohmann::json;

static void help() {
    cout
            << "\n------------------------------------------------------------------------------------\n"
            << " This program shows a 3D reconstruction of an object using the OpenCV SFM module\n"
            << " and PCL library, it also saves the point cloud from the reconstruction in a .ply file.\n"
            << " Usage:\n"
            << "        reconstruct <img_paths> <k_path> <cloud_path>\n"
            << "------------------------------------------------------------------------------------\n\n"
            << endl;
}


// To get paths to all images
static int getdir(const string _filename, vector<String> &files){
    ifstream myfile(_filename.c_str());
    if (!myfile.is_open()) {
        cout << "Unable to read file: " << _filename << endl;
        exit(0);
    } else {
        size_t found = _filename.find_last_of("/\\");
        string line_str, path_to_file = _filename.substr(0, found);
        while ( getline(myfile, line_str) )
            files.push_back(path_to_file+string("/")+line_str);
    }
    return 1;
}

// To read json file and get camera matrix
void get_k(string dir, Matx33d *K){
    json calib;
    ifstream file(dir);
    if (!file.is_open()) {
        cout << "Unable to read file: " << dir << endl;
        exit(0);
    }
    file >> calib;
    float fx  = calib["K"][0][0], fy  = calib["K"][1][1], cx = calib["K"][0][2], cy = calib["K"][1][2];
    *K = Matx33d( fx, 0, cx,
                  0, fy, cy,
                  0, 0,  1);
}

// To visualize Point Cloud
pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    // -----Open 3D viewer and add point cloud-----
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "Nube de puntos");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Nube de puntos");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}

int main(int argc, char* argv[])
{
    // Check if all input parameters are set
    if ( argc != 4 ){
        help();
        exit(0);
    }

    // Get paths for all images
    vector<String> images_paths;
    getdir( argv[1], images_paths );

    // Get camera matrix from json file
    Matx33d K;
    get_k(argv[2], &K);

    cout << "----------------------------" << endl;
    cout << "Intrinsics: " << endl << K << endl << endl;
    cout << "----------------------------" << endl;

    // Reconstruct 3d points from 2d images
    bool is_projective = true;
    vector<Mat> Rs_est, ts_est, points3d_estimated;
    reconstruct(images_paths, Rs_est, ts_est, K, points3d_estimated, is_projective);

    cout << "----------------------------" << endl;
    cout << "Reconstruction: " << endl;
    cout << "============================" << endl;
    cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
    cout << "Estimated cameras: " << Rs_est.size() << endl;
    cout << "Refined intrinsics: " << endl << K << endl << endl;
    cout << "----------------------------" << endl;

    // Convert 3D points stored in vector<Mat> to a pcl pointcloud
    vector<Point3f> pts;
    for(int i = 0; i < points3d_estimated.size(); ++i)
        pts.push_back(Point3f(points3d_estimated[i]));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize (pts.size());
    for (size_t i=0; i<pts.size(); i++) {
        (*cloud)[i].x = pts[i].x;
        (*cloud)[i].y = pts[i].y;
        (*cloud)[i].z = pts[i].z;
    }

    // Show point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = simpleVis(cloud);
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }

    // Save point cloud to .ply file
    string filePath = string(argv[3]) + "/cloud.ply";
    pcl::io::savePLYFileBinary(filePath, *cloud);

    return 0;
}

