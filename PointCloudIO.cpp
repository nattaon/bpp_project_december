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
	kinectraw_pointcloud.reset(new PointCloudXYZRGB);

}

void PointCloudIO::ReadKinectInput()
{
	rawrgbimge = kinect->get_colorframe(0.3);
	kinect->get_depthframe();
	kinect->mapping_pointcloud(kinectraw_pointcloud);

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

void PointCloudIO::LoadPointCloudToVariable(string filename, PointCloudXYZRGB::Ptr cloud)
{
	cloud.reset(new PointCloudXYZRGB);
	if (pcl::io::loadPCDFile(filename, *cloud) < 0)
	{
		cout << "Error loading model cloud." << endl;
	}
}

void PointCloudIO::SavePointCloud(string filename, PointCloudXYZRGB::Ptr cloud)
{
	if (cloud->points.size() <= 0)
	{
		//QMessageBox::information(0, QString("Save pointcloud"), QString("No pointcloud from kinect to save"), QMessageBox::Ok);
		cout << "No pointcloud to save." << endl;
		return;
	}
	
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->height = 1;

	pcl::io::savePCDFileBinary(filename, *cloud); //save-load faster


}

