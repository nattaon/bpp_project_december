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
void ViewerWindow::AddTextWindowCloudViewer(PointTypeXYZRGB position_OBB, Eigen::Vector3f major_vector, 
	double r, double g, double b, string drawtext, string cloudname)
{
	window_view->addText3D(drawtext, position_OBB, 0.05, r, g, b, cloudname);
}
void ViewerWindow::AddSymbolWindowCloudViewer(
	PointTypeXYZRGB position_OBB, 
	PointTypeXYZRGB min_point_OBB, 
	PointTypeXYZRGB max_point_OBB,
	Eigen::Vector3f mass_center,
	Eigen::Vector3f major_vector,
	Eigen::Vector3f middle_vector,
	double r, double g, double b, string cloudname)
{
	PointTypeXYZRGB position_major;
	PointTypeXYZRGB position_minor_plus, position_minor_minus;
	PointTypeXYZRGB position_center;
	

	double length_x = (max_point_OBB.x - min_point_OBB.x)*0.5;
	double length_y = (max_point_OBB.y - min_point_OBB.y)*0.5;
	double length_z = (max_point_OBB.z - min_point_OBB.z);
	double translate_pos = length_y*0.5;

	cout << "translate_pos= " << translate_pos << endl;

	position_major.x = (position_OBB.x + major_vector(0)*length_x);
	position_major.y = (position_OBB.y + major_vector(1)*length_y) + translate_pos;
	position_major.z = (position_OBB.z + major_vector(2)*length_z);

	position_minor_plus.x = (position_OBB.x + middle_vector(0)*length_x);
	position_minor_plus.y = (position_OBB.y + middle_vector(1)*length_y) + translate_pos;
	position_minor_plus.z = (position_OBB.z + middle_vector(2)*length_z);

	position_minor_minus.x = (position_OBB.x - middle_vector(0)*length_x);
	position_minor_minus.y = (position_OBB.y - middle_vector(1)*length_y) + translate_pos;
	position_minor_minus.z = (position_OBB.z - middle_vector(2)*length_z);

	position_center.x = position_OBB.x;
	position_center.y = position_OBB.y + translate_pos;
	position_center.z = position_OBB.z;


	window_view->addLine(position_center, position_minor_minus, 0.0f, 0.0f, 1.0f, cloudname + " minorlineminus");
	window_view->addLine(position_center, position_minor_plus, 0.0f, 0.0f, 1.0f, cloudname + " minorlineplus");

	window_view->addLine(position_center, position_major, 0.0f, 0.0f, 1.0f, cloudname + " majorline");
	window_view->addLine(position_minor_minus, position_minor_plus, 0.0f, 0.0f, 1.0f, cloudname + " minorline");
	window_view->addLine(position_minor_minus, position_major, 0.0f, 0.0f, 1.0f, cloudname + " minorminusline");
	window_view->addLine(position_minor_plus, position_major, 0.0f, 0.0f, 1.0f, cloudname + " minorplusline");
}

void ViewerWindow::ToggleAxisONWindowCloudViewer()
{
	window_view->addCoordinateSystem();
	window_view->spinOnce();
}
void ViewerWindow::ToggleAxisOFFWindowCloudViewer()
{
	window_view->removeCoordinateSystem();
	window_view->spinOnce();
}
