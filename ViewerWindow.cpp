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


void ViewerWindow::UpdateWindowCloudViewer(pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud)
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
