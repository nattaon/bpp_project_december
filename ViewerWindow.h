#ifndef VIEWERWINDOW_H
#define VIEWERWINDOW_H
#include "DataProcess.h"
#include "SharedHeader.h"

class ViewerWindow
{
public:
    ViewerWindow();
    void SetDataProcess(DataProcess* d);

	void UpdateWindowCloudViewer(PointCloudXYZRGB::Ptr pointcloud);
	void UpdateWindowRGB(cv::Mat image);

	

private:
    DataProcess *dataprocess;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> window_view;

};

#endif // VIEWERWINDOW_H
