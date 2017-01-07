#ifndef PointCloudOperation_H
#define PointCloudOperation_H
#include "SharedHeader.h"
#include "PlaneSegmentation.h"
#include "ClusterExtraction.h"
#include "ObjectTransformationData.h"
#include "OutlierRemoval.h"

class PointCloudOperation//:public PlaneSegmentation
{
public:
    PointCloudOperation();
	~PointCloudOperation();

	PlaneSegmentation *planeseg;
	ClusterExtraction *clusterextract;
	OutlierRemoval *outlierremove;
	VoxelGridFilter *voxelfilter;

	ObjectTransformationData *container;
	vector<ObjectTransformationData*> items;

	bool isSetAlignCorner;

	void SeparateContainerAndItems(vector<PointCloudXYZRGB::Ptr> extract_cluster_cloud);
	void ClearVectorItems();
	void AddLoadPointCloudToItems(PointCloudXYZRGB::Ptr cloud);
	void CalculateContainerTransformation();
	void CalculateItemsTransformation();

	void TranslatePointCloud(PointCloudXYZRGB::Ptr cloud, float translate_x, float translate_y, float translate_z);
	void MovePointCloudFromTo(PointCloudXYZRGB::Ptr cloud, PointTypeXYZRGB current_pos, PointTypeXYZRGB target_pos);
	void RotatePointCloudAtAxis(PointCloudXYZRGB::Ptr cloud,
		Eigen::Matrix<float, 1, 3>  floor_plane_normal_vector,
		Eigen::Matrix<float, 1, 3>  target_plane_normal_vector);
	void RotatePointCloud(PointCloudXYZRGB::Ptr cloud,
		float degree, Eigen::Matrix<float, 1, 3>  rotation_vector);


	void ApplyPassthroughFilter(PointCloudXYZRGB::Ptr cloud, 
		double xmin, double xmax,
		double ymin, double ymax,
		double zmin, double zmax);

	void SurfaceFillCloud(PointCloudXYZRGB::Ptr cloud, float leaf_size, float x_length, float y_length, float z_length);
	void DuplicateInvertCloud(PointCloudXYZRGB::Ptr cloud, float x_length, float y_length, float z_length);

private:



};

#endif // PointCloudOperation_H