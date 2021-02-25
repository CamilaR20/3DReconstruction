# 3D Reconstruction from 2D pictures

This project uses the OpenCV SFM module to reconstruct an object from multiple 2D images and PCL to process the point cloud. It uses CMAKE to compile and it has 2 targets that can be run:
  - reconstruct
  - process_pcl
  
## Build 🛠️
To build use CMake minimum required 3.17.

### Prerequisites 
  - OpenCV 4.5.0 
  - Ceres 1.14.0
  - PCL 1.11.1
  - nlohmann/json 3.9.1
  
* Set your own OpenCV_DIR, PCL_DIR, Ceres_DIR and nlohmann_json_DIR in CMakeList.txt e.g: **/usr/local/Cellar/opencv/4.5.0_2/lib/cmake/opencv4**.

In macOS the packages can be installed with Homebrew, e.g: 
```
brew install opencv@4.5.0
```

## Usage 💻 
### Reconstruct
This program shows a 3D reconstruction of an object using the OpenCV SFM module and PCL library, it also saves the point cloud from the reconstruction in a .ply file.
Usage:
```
reconstruct <img_paths> <k_path> <cloud_path>
```
Where:
  - **img_paths** is an absolute path to a txt file which contains the list of images to use for reconstruction.
  - **k_path** is an absolute path to a json file which contains the camera matrix.
  - **cloud_path** is an absolute path to the folder where you want to save the point cloud in a .ply file.
  
### Process point cloud
This program loads a point cloud from a .ply file, filters it with an Statistical Outlier filter, computes the normals and uses a reconstruction method to create a mesh using PCL.
Usage:
```
process_pcl <cloud_path> <mesh_path> <show> <normals_method> <reconstruction_method>
```
Where:
  - **ply_path** is an absolute path to the .ply file which contains the point cloud.
  - **mesh_path** is an absolute path to the folder where you want to save the mesh in a .ply file.
  - **show** can be 'y' if you want the program to open a viewer and show the point cloud after loading it and after filtering it.
  - **normals_method** can be 'n' or 'mls' to use smoothing and normal estimation based on polynomial reconstruction.
  - **reconstruction_method** can be 'p'if you want to use Poisson, 'gt' for Greedy Triangulation or 'gp' for Grid Projection.
  

## Authors ✒️
* [CamilaR20](https://github.com/CamilaR20)
* [santiro99](https://github.com/santiro99)
* [elianaromero](https://github.com/elianaromero)
