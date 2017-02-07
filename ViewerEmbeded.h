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

	int POINT_SIZE;
	void SetPointsize(int pt);
	void ClearPointCloudEmbededCloudViewer();
	void UpdateCloudViewer(PointCloudXYZRGB::Ptr pointcloud);
	void DrawXYZAxis();
	void RemoveXYZAxis();
	void RemoveBounding();
	void DrawBounding(float w, float h, float d,
		float x, float y, float z,
		float r, float g, float b);

};

#endif // VIEWEREMBEDED_H