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

	void ClearPointCloudEmbededCloudViewer();
	void UpdateCloudViewer(bool draw_axis, bool draw_bounding, PointCloudXYZRGB::Ptr pointcloud);
	void DrawXYZAxis();
	void AddItemCube(float w, float h, float d,
		float x, float y, float z,
		float r, float g, float b,
		string shapename);

};

#endif // VIEWEREMBEDED_H