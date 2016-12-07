#ifndef PointCloudOperation_H
#define PointCloudOperation_H
#include "SharedHeader.h"
#include "PlaneSegmentation.h"

class PointCloudOperation//:public PlaneSegmentation
{
public:
    PointCloudOperation();
	~PointCloudOperation();

	PlaneSegmentation *planeseg;
	//void PlaneSegmentCloud(double threshold, pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud);
};

#endif // PointCloudOperation_H