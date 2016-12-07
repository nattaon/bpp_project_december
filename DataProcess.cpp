#include "DataProcess.h"
DataProcess::DataProcess()
{
	cout << "DataProcess()" << endl;
	//new KinectV2Connector() when call ConnectKinect()
	rawpointcloud.reset(new PointCloudXYZRGB);
	isKinectRunning = false;
}
DataProcess::~DataProcess()
{
	if (isKinectRunning) delete kinect;
}

void DataProcess::ConnectKinect()
{
	kinect = new KinectV2Connector();
	int return_val = kinect->openKinect();

	isKinectRunning = true;
}

void DataProcess::ReadKinectInput()
{
	rawrgbimge = kinect->get_colorframe(0.3);
	kinect->get_depthframe();
	kinect->mapping_pointcloud(rawpointcloud);

}
PointCloudXYZRGB::Ptr DataProcess::GetKinectPointCloud()
{
	return rawpointcloud;
}
cv::Mat DataProcess::GetKinectRGBImage()
{
	return rawrgbimge;
}

void DataProcess::DisconnectKinect()
{
	if (!isKinectRunning) return;

	delete kinect;
	isKinectRunning = false;
}

void DataProcess::LoadPointCloud(string filename)
{
	loadedpointcloud.reset(new PointCloudXYZRGB);
	if (pcl::io::loadPCDFile(filename, *loadedpointcloud) < 0)
	{
		cout << "Error loading model cloud." << endl;
	}
}

void DataProcess::SavePointCloud(string filename)
{
	if (rawpointcloud->points.size() <= 0)
	{
		//QMessageBox::information(0, QString("Save pointcloud"), QString("No pointcloud from kinect to save"), QMessageBox::Ok);
		cout << "No pointcloud from kinect to save." << endl;
		return;
	}

	rawpointcloud->width = rawpointcloud->points.size();
	rawpointcloud->height = 1;

	pcl::io::savePCDFileBinary(filename, *rawpointcloud); //save-load faster


}

PointCloudXYZRGB::Ptr DataProcess::GetLoadedPointCloud()
{

	return loadedpointcloud;
}


