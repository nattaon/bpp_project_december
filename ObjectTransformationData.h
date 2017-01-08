#ifndef OBJECTTRANSFORMATIONDATA_H
#define OBJECTTRANSFORMATIONDATA_H
#include "SharedHeader.h"
#include "PointCloudTransformationExtraction.h"

class ObjectTransformationData
{
public:
    ObjectTransformationData();
	void CalculateWDH();
	void SetLengthMM(int box_x_dim, int box_y_dim, int box_z_dim);
	void SetTransformMinMaxPos(PointTypeXYZRGB items_min_pos, PointTypeXYZRGB items_max_pos);
	void SetTransformVector(Eigen::Vector3f items_major_vector,
		Eigen::Vector3f items_middle_vector,
		Eigen::Vector3f items_minor_vector);


	PointCloudXYZRGB::Ptr object_pointcloud;
	PointCloudTransformationExtraction *transform;
	
	PointTypeXYZRGB target_orientation;
	PointTypeXYZRGB target_position;
	PointTypeXYZRGB input_dimension;

	
	int x_length_mm, y_length_mm, z_length_mm;
	double x_length, y_length, z_length;

	double r, g, b;
	int rotation_case;
	int packing_order;
};

#endif // OBJECTTRANSFORMATIONDATA_H