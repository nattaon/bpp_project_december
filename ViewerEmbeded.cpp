#include "ViewerEmbeded.h"

ViewerEmbeded::ViewerEmbeded(QVTKWidget *widget)
{
	ui_widget_viewer = widget;

	//	boost::shared_ptr<pcl::visualization::PCLVisualizer> embeded_view;boost::shared_ptr<pcl::visualization::PCLVisualizer> embeded_view;
	embeded_view.reset(new pcl::visualization::PCLVisualizer("embeded_view", false));
	embeded_view->setBackgroundColor(0, 0, 0);
	//embeded_view->addCoordinateSystem();
	embeded_view->initCameraParameters();
	ui_widget_viewer->SetRenderWindow(embeded_view->getRenderWindow());
	embeded_view->setupInteractor(ui_widget_viewer->GetInteractor(), ui_widget_viewer->GetRenderWindow());


	//DrawXYZAxis();

}
ViewerEmbeded::~ViewerEmbeded()
{

}
void ViewerEmbeded::SetPointsize(int pt)
{
	cout << "ViewerEmbeded::SetPointsize " << pt << endl;
	POINT_SIZE = pt;
}
void ViewerEmbeded::ClearPointCloudEmbededCloudViewer()
{
	embeded_view->removeAllPointClouds();
	ui_widget_viewer->update();
}

void ViewerEmbeded::UpdateCloudViewer(PointCloudXYZRGB::Ptr pointcloud)
{

	if (!embeded_view->updatePointCloud(pointcloud, "embeded_view"))
	{
		embeded_view->addPointCloud(pointcloud, "embeded_view");
	}
	embeded_view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, "embeded_view");


	ui_widget_viewer->update();
}


void ViewerEmbeded::DrawXYZAxis()
{
	pcl::PointXYZ center(0, 0, 0);
	pcl::PointXYZ x_axis(1, 0, 0);
	pcl::PointXYZ y_axis(0, 1, 0);
	pcl::PointXYZ z_axis(0, 0, 1);
	embeded_view->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "x_axis");
	embeded_view->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "y_axis");
	embeded_view->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "z_axis");

	ui_widget_viewer->update();
}
void ViewerEmbeded::RemoveXYZAxis()
{
	embeded_view->removeShape("x_axis");
	embeded_view->removeShape("y_axis");
	embeded_view->removeShape("z_axis");
}

void ViewerEmbeded::RemoveBounding()
{
	embeded_view->removeShape("bounding");
}


void ViewerEmbeded::DrawBounding(float w, float h, float d,
	float x, float y, float z,
	float r, float g, float b)
{


	float xmin = x;
	float ymin = y;
	float zmin = z;
	float xmax = (x + w);
	float ymax = (y + h);
	float zmax = (z + d);

	//cout << endl;
	//cout << r << "," << g << "," << b << endl; 
	//cout << "x " << xmin << "," << xmax << endl;
	//cout << "y " << ymin << "," << ymax << endl;
	//cout << "z " << zmin << "," << zmax << endl;

	embeded_view->removeShape("bounding");
	embeded_view->addCube(xmin, xmax, ymin, ymax, zmin, zmax, r, g, b, "bounding");
}


