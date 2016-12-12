#ifndef CLUSTEREXTRACTION_H
#define CLUSTEREXTRACTION_H
#include "SharedHeader.h"

class ClusterExtraction
{
public:
	ClusterExtraction();
	~ClusterExtraction();

	

	void SetClusterExtractValue(
		double cluster_tolerance, int cluster_min_size, int cluster_max_size);
	void ShowClusterInColor(PointCloudXYZRGB::Ptr cloud);

	vector<PointCloudXYZRGB::Ptr> GetExtractCluster();

	PointCloudXYZRGB::Ptr applied_colorcluster_cloud;

private:
	double tolerance;
	int min_size, max_size;

	
	vector<PointCloudXYZRGB::Ptr> array_cluster_cloud;
	PointCloudXYZRGB::Ptr cluster_cloud_no_filter;
	int randomcolor();

};

#endif
