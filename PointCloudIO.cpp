#include "PointCloudIO.h"

PointCloudIO::PointCloudIO()
{
	cout << "PointCloudIO()" << endl;
	isKinectRunning = false;
}

PointCloudIO::~PointCloudIO()
{
	if (isKinectRunning) delete kinect;
}

void PointCloudIO::ConnectKinect()
{
	kinect = new KinectV2Connector();
	int return_val = kinect->openKinect();

	isKinectRunning = true;
	raw_pointcloud.reset(new PointCloudXYZRGB);

}

void PointCloudIO::ReadKinectInput()
{
	rawrgbimge = kinect->get_colorframe(0.3);
	kinect->get_depthframe();
	kinect->mapping_pointcloud(raw_pointcloud);

}

void PointCloudIO::DisconnectKinect()
{
	if (!isKinectRunning) return;

	delete kinect;
	isKinectRunning = false;
}


void PointCloudIO::LoadPointCloud(string filename)
{
	loaded_pointcloud.reset(new PointCloudXYZRGB);
	if (pcl::io::loadPCDFile(filename, *loaded_pointcloud) < 0)
	{
		cout << "Error loading model cloud." << endl;
	}
}

void PointCloudIO::SavePointCloud(string filename)
{
	if (raw_pointcloud->points.size() <= 0)
	{
		//QMessageBox::information(0, QString("Save pointcloud"), QString("No pointcloud from kinect to save"), QMessageBox::Ok);
		cout << "No pointcloud from kinect to save." << endl;
		return;
	}

	raw_pointcloud->width = raw_pointcloud->points.size();
	raw_pointcloud->height = 1;

	pcl::io::savePCDFileBinary(filename, *raw_pointcloud); //save-load faster


}