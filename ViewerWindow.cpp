#include "ViewerWindow.h"

ViewerWindow::ViewerWindow()
{
	cout << "ViewerWindow()" << endl;

	cv::namedWindow("kinect_rgb");
	cvMoveWindow("kinect_rgb", 0, 0);

	window_view.reset(new pcl::visualization::PCLVisualizer("window_view"));
	window_view->setPosition(50,0);
	window_view->setBackgroundColor(0, 0, 0);
	window_view->initCameraParameters();
}

void ViewerWindow::SetDataProcess(DataProcess* d) {dataprocess = d;}


void ViewerWindow::UpdateWindowCloudViewer(PointCloudXYZRGB::Ptr pointcloud)
{
	if (!window_view->updatePointCloud(pointcloud, "window_view"))
	{
		window_view->addPointCloud(pointcloud, "window_view");
	}
	window_view->spinOnce();
}
void ViewerWindow::UpdateWindowRGB(cv::Mat image)
{
	cv::imshow("kinect_rgb", image);
}

void ViewerWindow::ClearPointCloudWindowCloudViewer()
{
	window_view->removeAllPointClouds();
}
void ViewerWindow::ClearShapeWindowCloudViewer()
{
	window_view->removeAllShapes();
}

void ViewerWindow::AddBoundingBoxWindowCloudViewer(PointTypeXYZRGB position_OBB, 
	PointTypeXYZRGB min_point_OBB, PointTypeXYZRGB max_point_OBB, 
	Eigen::Matrix3f rotational_matrix_OBB, string cloudname)
{
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	window_view->addCube(position, quat, 
		max_point_OBB.x - min_point_OBB.x, 
		max_point_OBB.y - min_point_OBB.y, 
		max_point_OBB.z - min_point_OBB.z, cloudname);

}

void ViewerWindow::AddVectorDirectionWindowCloudViewer(Eigen::Vector3f mass_center, 
	Eigen::Vector3f major_vector, Eigen::Vector3f middle_vector, Eigen::Vector3f minor_vector, 
	string cloudname)
{
	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	window_view->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, cloudname + " major eigen vector");
	window_view->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, cloudname + " middle eigen vector");
	window_view->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, cloudname + " minor eigen vector");
}

