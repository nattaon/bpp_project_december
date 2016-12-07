#include "DataProcess.h"
DataProcess::DataProcess()
{
	cout << "DataProcess()" << endl;
	//new KinectV2Connector() when call ConnectKinect()
	
	//cloudio = new PointCloudIO();
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


