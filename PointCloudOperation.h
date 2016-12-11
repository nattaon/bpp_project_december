#ifndef PointCloudOperation_H
#define PointCloudOperation_H
#include "SharedHeader.h"
#include "PlaneSegmentation.h"
#include "ClusterExtraction.h"

class PointCloudOperation//:public PlaneSegmentation
{
public:
    PointCloudOperation();
	~PointCloudOperation();

	PlaneSegmentation *planeseg;
	ClusterExtraction *clusterextract;
	//void PlaneSegmentCloud(double threshold, pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud);
};

#endif // PointCloudOperation_H