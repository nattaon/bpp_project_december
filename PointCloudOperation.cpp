#include "PointCloudOperation.h"

PointCloudOperation::PointCloudOperation()
{
	cout << "PointCloudOperation()" << endl;
	
	planeseg = new PlaneSegmentation();
}

PointCloudOperation::~PointCloudOperation()
{
	delete planeseg;
}

/*
void PointCloudOperation::PlaneSegmentCloud(double threshold, pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud)
{


}*/