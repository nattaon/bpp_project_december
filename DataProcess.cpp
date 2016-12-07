#include "DataProcess.h"
DataProcess::DataProcess()
{
	cout << "DataProcess()" << endl;
	//new KinectV2Connector() when call ConnectKinect()
	
	//cloudio = new PointCloudIO();
	currentdisplay_pointcloud.reset(new PointCloudXYZRGB);
}
DataProcess::~DataProcess()
{

}


PointCloudXYZRGB::Ptr DataProcess::GetKinectPointCloud()
{
	return raw_pointcloud;
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


PointCloudXYZRGB::Ptr DataProcess::GetAppliedRedPlanePointCloud()
{
	return planeseg->applied_redplane_cloud;
}
PointCloudXYZRGB::Ptr DataProcess::GetRemovedPlanePointCloud()
{
	return planeseg->removed_plane_cloud;
}

