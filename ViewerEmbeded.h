#ifndef VIEWEREMBEDED_H
#define VIEWEREMBEDED_H
#include "SharedHeader.h"

class ViewerEmbeded
{
public:
	ViewerEmbeded(QVTKWidget *widget);
	~ViewerEmbeded();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> embeded_view;
	QVTKWidget *ui_widget_viewer;

	void UpdateCloudViewer(PointCloudXYZRGB::Ptr pointcloud);
	void DrawXYZAxis();

};

#endif // VIEWEREMBEDED_H