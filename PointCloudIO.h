#ifndef POINTCLOUDIO_H
#define POINTCLOUDIO_H
#include "SharedHeader.h"
#include "KinectV2Connector.h"

class PointCloudIO
{
public:
    PointCloudIO();
	~PointCloudIO();

	void ConnectKinect();
	void ReadKinectInput();
	void DisconnectKinect();

	void LoadPointCloud(string filename);
	void SavePointCloud(string filename, PointCloudXYZRGB::Ptr cloud);

private:
	KinectV2Connector *kinect;
	bool isKinectRunning;

protected:
	PointCloudXYZRGB::Ptr raw_pointcloud;
	PointCloudXYZRGB::Ptr loaded_pointcloud;
	cv::Mat rawrgbimge;


};

#endif // POINTCLOUDIO_H