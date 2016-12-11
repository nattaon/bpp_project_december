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
	void ShowClusterBoundingbox(PointCloudXYZRGB::Ptr cloud);
	std::vector<PointCloudXYZRGB::Ptr> ExtractCluster();

private:
	double tolerance;
	int min_size, max_size;

	PointCloudXYZRGB::Ptr cloud_cluster_no_filter;
	std::vector<PointCloudXYZRGB::Ptr> array_cluster_cloud;

	int randomcolor();

};

#endif
