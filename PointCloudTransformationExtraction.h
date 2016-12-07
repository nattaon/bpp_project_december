#ifndef POINTCLOUDTRANSFORMATIONEXTRACTION_H
#define POINTCLOUDTRANSFORMATIONEXTRACTION_H
#include "SharedHeader.h"
#include "VoxelGridFilter.h"

class PointCloudTransformationExtraction:public VoxelGridFilter
{
public:
    PointCloudTransformationExtraction();
	void CalculateTransformation(PointCloudXYZRGB::Ptr cloud);
	void CalculateWDH();

	Eigen::Vector3f mass_center;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	PointTypeXYZRGB min_point_OBB;
	PointTypeXYZRGB max_point_OBB;
	PointTypeXYZRGB position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;

	double width, depth, height;

private:
	pcl::MomentOfInertiaEstimation <PointTypeXYZRGB> feature_extractor;

//protected:



};

#endif // POINTCLOUDTRANSFORMATIONEXTRACTION_H