#ifndef PointCloudOperation_H
#define PointCloudOperation_H
#include "SharedHeader.h"
#include "PlaneSegmentation.h"
#include "ClusterExtraction.h"
#include "ObjectTransformationData.h"

class PointCloudOperation//:public PlaneSegmentation
{
public:
    PointCloudOperation();
	~PointCloudOperation();

	PlaneSegmentation *planeseg;
	ClusterExtraction *clusterextract;

	ObjectTransformationData *container;
	vector<ObjectTransformationData> items;

	void SeparateContainerAndItems(vector<PointCloudXYZRGB::Ptr> extract_cluster_cloud);
	void CalculateContainerTransformation();
	void CalculateItemsTransformation();

	void MovePointCloudFromTo(PointCloudXYZRGB::Ptr cloud, PointTypeXYZRGB current_pos, PointTypeXYZRGB target_pos);
	void RotatePointCloudAtAxis(PointCloudXYZRGB::Ptr cloud,
		Eigen::Matrix<float, 1, 3>  floor_plane_normal_vector,
		Eigen::Matrix<float, 1, 3>  target_plane_normal_vector);

private:



};

#endif // PointCloudOperation_H