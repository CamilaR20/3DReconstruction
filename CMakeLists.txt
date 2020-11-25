cmake_minimum_required(VERSION 3.17)
project(3DReconstruction)

set(CMAKE_CXX_STANDARD 14)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

#set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(OpenCV_DIR /usr/local/Cellar/opencv/4.5.0_2/lib/cmake/opencv4)
set(Ceres_DIR /usr/local/Cellar/ceres-solver/1.14.0_13/lib/cmake/Ceres)
set(PCL_DIR /usr/local/Cellar/pcl/1.11.1_5/share/pcl-1.11)
set(nlohmann_json_DIR /usr/local/Cellar/nlohmann-json/3.9.1_1)

set(CMAKE_MODULE_PATH ${PCL_DIR}/../Modules)

##############################################################################
# PACKAGES
##############################################################################
message("Finding packages...")

message("***********************")
message("OpenCV PACKAGE")
message("***********************")

set(OpenCV_Components core highgui features2d xfeatures2d calib3d imgcodecs imgproc sfm)
find_package(OpenCV REQUIRED PATHS ${OPENCV_DIR} COMPONENTS ${OpenCV_Components})
if(OpenCV_FOUND)
    message(STATUS "OpenCV status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV ${OpenCV_VERSION} not found")
endif()


message("***********************")
message("Ceres PACKAGE")
message("***********************")

find_package(Ceres 1.14.0 REQUIRED)
if(CERES_FOUND)
    message(STATUS "Ceres library status:")
    message(STATUS "    version: ${CERES_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: Ceres ${CERES_VERSION} not found")
endif()

message("***********************")
message("PCL PACKAGE")
message("***********************")

find_package(PCL 1.11.1 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
if(PCL_FOUND)
    message(STATUS "PCL library status:")
    message(STATUS "    version: ${PCL_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: PCL not found")
endif()

message("***********************")
message("nlohmann_json PACKAGE")
message("***********************")

find_package(nlohmann_json REQUIRED)
add_definitions(${nlohmann_json_DEFINITIONS})
if(nlohmann_json_FOUND)
    message(STATUS "nlohmann_json library status:")
    message(STATUS "    version: ${nlohmann_json_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: nlohmann_json not found")
endif()
##############################################################################
# HEADERS
##############################################################################
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${nlohmann_json_INCLUDE_DIRS})
include(CheckFunctionExists)


##############################################################################
# LIBRARIES PATH
##############################################################################
link_directories(${OpenCV_LIBRARIES})
link_directories(${CERES_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${nlohmann_json_LIBRARIES})
message(STATUS "Directorios: ${PCL_DIR}")

# specify the executable target to be built
add_executable(main main.cpp)
add_executable(reconstruct reconstruct.cpp)
add_executable(process process_pcl.cpp)

set(OpenCV_LIBRARIES opencv_core opencv_highgui opencv_features2d opencv_calib3d
        opencv_imgproc opencv_imgcodecs opencv_xfeatures2d opencv_sfm)

# link the executable target against libraries
target_link_libraries(main ${PCL_LIBRARIES})
target_link_libraries(reconstruct ${nlohmann_json_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(process ${PCL_LIBRARIES})


