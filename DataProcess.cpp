#include "DataProcess.h"
DataProcess::DataProcess()
{
	cout << "DataProcess()" << endl;
	//new KinectV2Connector() when call ConnectKinect()
	
	//cloudio = new PointCloudIO();
	currentdisplay_pointcloud.reset(new PointCloudXYZRGB);
	lastedoperate_pointcloud.reset(new PointCloudXYZRGB);
}
DataProcess::~DataProcess()
{

}


PointCloudXYZRGB::Ptr DataProcess::GetKinectPointCloud()
{
	return kinectraw_pointcloud;
}
cv::Mat DataProcess::GetKinectRGBImage()
{
	return rawrgbimge;
}
PointCloudXYZRGB::Ptr DataProcess::GetLoadedPointCloud()
{

	return loaded_pointcloud;
}
void DataProcess::SetCurrentDisplayPointCloud(PointCloudXYZRGB::Ptr cloud)
{
	pcl::copyPointCloud(*cloud, *currentdisplay_pointcloud);
	//currentdisplay_pointcloud = cloud;
}

PointCloudXYZRGB::Ptr DataProcess::GetCurrentDisplayPointCloud()
{
	return currentdisplay_pointcloud;
}
PointCloudXYZRGB::Ptr DataProcess::GetLastedOperateDisplayPointCloud()
{
	return lastedoperate_pointcloud;
}
int DataProcess::GetCurrentDisplayPointCloudSize()
{
	return static_cast<int>(currentdisplay_pointcloud->size());
}

PointCloudXYZRGB::Ptr DataProcess::GetAppliedRedPlanePointCloud()
{
	return planeseg->applied_redplane_cloud;
}
PointCloudXYZRGB::Ptr DataProcess::GetRemovedPlanePointCloud()
{
	return planeseg->removed_plane_cloud;
}
PointCloudXYZRGB::Ptr DataProcess::GetRemovedPlaneOutsidePointCloud()
{
	return planeseg->removed_planeoutside_cloud;
}
PointCloudXYZRGB::Ptr DataProcess::GetOnlyPlanePointCloud()
{
	return planeseg->only_plane_cloud;
}
PointCloudXYZRGB::Ptr DataProcess::GetColoredClusterPointCloud()
{
	return clusterextract->color_cluster_cloud;
}
void DataProcess::StoreLastedOperationCloud(PointCloudXYZRGB::Ptr cloud)
{
	pcl::copyPointCloud(*cloud, *lastedoperate_pointcloud);
}




