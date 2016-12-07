#ifndef POINTCLOUDPROCESS_H
#define POINTCLOUDPROCESS_H
#include "SharedHeader.h"
#include "PlaneSegmentation.h"

class PointCloudProcess:public PlaneSegmentation
{
public:
    PointCloudProcess();
	//void PlaneSegmentCloud(double threshold, pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud);
};

#endif // POINTCLOUDPROCESS_H