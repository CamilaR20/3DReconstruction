# 3D Reconstruction from 2D pictures

This project uses the OpenCV SFM module to reconstruct an object from multiple 2D images and PCL to process the point cloud. It uses CMAKE to compile and it has 3 targets that can be run:
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
  - **img_paths** is a txt file absolute path which contains the list of images to use for reconstruction.
  - **k_path** is a json file absolute path which contains the camera matrix.
  - **cloud_path** is an absolute path to where you want to save the point cloud in a .ply file.
  
### Process point cloud
This program loads a point cloud from a .ply file and ...
Usage:
```
process_pcl <ply_path>
```
Where:
  - **ply_path** is an absolute path to the .ply file.

## Authors ✒️
* [CamilaR20](https://github.com/CamilaR20)
* [santiro99](https://github.com/santiro99)
