#ifndef DATAPROCESS_H
#define DATAPROCESS_H
#include "SharedHeader.h"
#include "KinectV2Connector.h"

class DataProcess
{
public:
    DataProcess();
	~DataProcess();
	void ConnectKinect();
	void ReadKinectInput();
	void DisconnectKinect();
	PointCloudXYZRGB::Ptr GetKinectPointCloud();
	cv::Mat GetKinectRGBImage();
	void LoadPointCloud(string filename);
	void SavePointCloud(string filename);
	PointCloudXYZRGB::Ptr GetLoadedPointCloud();
	

private:
	KinectV2Connector *kinect;
	bool isKinectRunning;

	PointCloudXYZRGB::Ptr rawpointcloud;
	PointCloudXYZRGB::Ptr loadedpointcloud;
	cv::Mat rawrgbimge;



};

#endif // DATAPROCESS_H
