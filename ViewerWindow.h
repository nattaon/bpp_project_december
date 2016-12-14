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
	void AddTextWindowCloudViewer(PointTypeXYZRGB point_position, double text_scale,
		double r, double g, double b, string drawtext, string cloudname);
	void AddSphereWindowCloudViewer(PointTypeXYZRGB point_position, double radius, double r, double g, double b, string id_name);


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

	pcl::PolygonMesh visualizerGetCameraMesh(const Eigen::Matrix3f& R, const Eigen::Vector3f& t, 
		float r, float g, float b,
		Eigen::Vector3f& vforward, Eigen::Vector3f& rgb);

private:
    DataProcess *dataprocess;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> window_view;

};

#endif // VIEWERWINDOW_H
