#ifndef SHAREDHEADER_H
#define SHAREDHEADER_H

//#include "vld.h"
//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>

#include "build/ui_MainUI.h"

#include <iostream>
#include <Windows.h>
#include <Kinect.h>

#include <opencv2/opencv.hpp>  //show color image from kinect
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Qt
#include <QApplication>
#include <QMainWindow>
#include <QTime>
#include <QLineEdit>
#include <QTreeWidgetItem>
#include <QMessageBox>
#include <QFileDialog>
#include <QKeyEvent>

// Point Cloud Library
#include <pcl/common/common_headers.h> 
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>

#include <pcl/correspondence.h>

//#include <pcl/features/board.h>
//#include <pcl/features/intensity_gradient.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
//#include <pcl/features/rift.h>
//#include <pcl/features/shot_omp.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
//#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/recognition/cg/geometric_consistency.h>
//#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/ros/conversions.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
//#include <boost/thread/thread.hpp>

#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


typedef pcl::PointXYZRGB PointTypeXYZRGB;
typedef pcl::PointCloud<PointTypeXYZRGB> PointCloudXYZRGB;

using namespace std;


#endif