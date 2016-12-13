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

	void ClearPointCloudWindowCloudViewer();
	void ClearShapeWindowCloudViewer();

	void AddBoundingBoxWindowCloudViewer(PointTypeXYZRGB position_OBB,
		PointTypeXYZRGB min_point_OBB, PointTypeXYZRGB max_point_OBB,
		Eigen::Matrix3f rotational_matrix_OBB, string cloudname);
	void AddVectorDirectionWindowCloudViewer(Eigen::Vector3f mass_center,
		Eigen::Vector3f major_vector, Eigen::Vector3f middle_vector, Eigen::Vector3f minor_vector,
		string cloudname);
	void AddTextWindowCloudViewer(PointTypeXYZRGB position_OBB, Eigen::Vector3f major_vector,
		double r, double g, double b, string drawtext, string cloudname);
	void AddSymbolWindowCloudViewer(
		PointTypeXYZRGB position_OBB,
		PointTypeXYZRGB min_point_OBB,
		PointTypeXYZRGB max_point_OBB,
		Eigen::Vector3f mass_center,
		Eigen::Vector3f major_vector,
		Eigen::Vector3f minor_vector,
		double r, double g, double b, string cloudname);

	void ToggleAxisONWindowCloudViewer();
	void ToggleAxisOFFWindowCloudViewer();

private:
    DataProcess *dataprocess;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> window_view;

};

#endif // VIEWERWINDOW_H
