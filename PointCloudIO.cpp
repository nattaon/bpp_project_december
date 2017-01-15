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

PointCloudXYZRGB::Ptr PointCloudIO::LoadPcdFileToPointCloudVariable(string filename)
{
	pcl::PCLPointCloud2 cloud_blob;
	PointCloudXYZRGB::Ptr cloud;
	cloud.reset(new PointCloudXYZRGB);

	size_t index_pcd = -1;	
	size_t index_ply = -1;

	index_pcd = filename.find("pcd");
	index_ply = filename.find("ply");

	cout << "index_pcd=" << index_pcd << endl;
	cout << "index_ply=" << index_ply << endl;
	
	boost::filesystem::path p(filename.c_str());
	std::string extension = p.extension().string();
	int result = -1;
	if (extension == ".pcd")
		result = pcl::io::loadPCDFile(filename, cloud_blob);
	else if (extension == ".ply")
		result = pcl::io::loadPLYFile(filename, cloud_blob);

	//sensor_msgs::PointCloud2::fields
	//cout << "filed=" << cloud_blob.fields() << endl;

	//bool curvature_available = pcl::traits::has_field<PointT, pcl::fields::curvature>::value;
	//bool rgb = FieldMatches<pcl::PCLPointCloud2>

	pcl::fromPCLPointCloud2(cloud_blob, *cloud);

	if (result == -1)
		return NULL;
	else
		return cloud;


	/*

	if (index_pcd != std::string::npos)//pcd file
	{
		if (pcl::io::loadPCDFile(filename, *cloud) < 0)
		{
			cout << "Error loading model cloud." << endl;
			return NULL;
		}
	}
	else if (index_ply != std::string::npos)//ply file
	{
		if (pcl::io::loadPLYFile(filename, *cloud) < 0)
		{
			cout << "Error loading model cloud." << endl;
			return NULL;
		}
	}*/

	return cloud;
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

