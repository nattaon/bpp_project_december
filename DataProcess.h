#ifndef DATAPROCESS_H
#define DATAPROCESS_H
#include "SharedHeader.h"
#include "PointCloudIO.h"
#include "PointCloudProcess.h"
#include "TextFileIO.h"

class DataProcess:public PointCloudIO, public PointCloudProcess
{
public:
    DataProcess();
	~DataProcess();

	//PointCloudIO *cloudio; // just declare, this not call PointCloudIO::PointCloudIO()
	//PointCloudProcess *cloudprocess; // just declare, this not call PointCloudProcess::PointCloudProcess()
	//TextFileIO *fileio; // just declare, this not call TextFileIO::TextFileIO()

	PointCloudXYZRGB::Ptr GetKinectPointCloud();
	cv::Mat GetKinectRGBImage();
	PointCloudXYZRGB::Ptr GetLoadedPointCloud();

	void SetCurrentDisplayPointCloud(PointCloudXYZRGB::Ptr cloud);
	PointCloudXYZRGB::Ptr GetCurrentDisplayPointCloud();

	PointCloudXYZRGB::Ptr GetAppliedRedPlanePointCloud();
	PointCloudXYZRGB::Ptr GetRemovedPlanePointCloud();
	

private:
	PointCloudXYZRGB::Ptr currentdisplay_pointcloud;




};

#endif // DATAPROCESS_H
