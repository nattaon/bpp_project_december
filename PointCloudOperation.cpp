#include "PointCloudOperation.h"

PointCloudOperation::PointCloudOperation()
{
	cout << "PointCloudOperation()" << endl;
	
	planeseg = new PlaneSegmentation();
	clusterextract = new ClusterExtraction();
}

PointCloudOperation::~PointCloudOperation()
{
	delete planeseg;
	delete clusterextract;
}

/*
void PointCloudOperation::PlaneSegmentCloud(double threshold, pcl::PointCloud<PointTypeXYZRGB>::Ptr pointcloud)
{


}*/