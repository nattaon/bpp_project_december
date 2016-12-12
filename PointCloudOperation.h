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

private:
	void CalculateTransformation();


};

#endif // PointCloudOperation_H