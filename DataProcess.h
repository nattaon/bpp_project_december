#ifndef DATAPROCESS_H
#define DATAPROCESS_H
#include "SharedHeader.h"
#include "PointCloudIO.h"
#include "PointCloudOperation.h"
#include "TextFileIO.h"
#include "CalculateBppErhan.h"
class DataProcess :public PointCloudIO, public PointCloudOperation, public TextFileIO, public CalculateBppErhan
{
public:
    DataProcess();
	~DataProcess();

	PointCloudXYZRGB::Ptr GetKinectPointCloud();
	cv::Mat GetKinectRGBImage();
	PointCloudXYZRGB::Ptr GetLoadedPointCloud();

	void SetCurrentDisplayPointCloud(PointCloudXYZRGB::Ptr cloud);
	PointCloudXYZRGB::Ptr GetCurrentDisplayPointCloud();
	int GetCurrentDisplayPointCloudSize();

	PointCloudXYZRGB::Ptr GetAppliedRedPlanePointCloud();
	PointCloudXYZRGB::Ptr GetRemovedPlanePointCloud();
	PointCloudXYZRGB::Ptr GetRemovedPlaneOutsidePointCloud();
	PointCloudXYZRGB::Ptr GetOnlyPlanePointCloud();
	PointCloudXYZRGB::Ptr GetColoredClusterPointCloud();

	void StoreLastedOperationCloud(PointCloudXYZRGB::Ptr cloud);



private:
	PointCloudXYZRGB::Ptr currentdisplay_pointcloud;
	PointCloudXYZRGB::Ptr lastedoperate_pointcloud;




};

#endif // DATAPROCESS_H
