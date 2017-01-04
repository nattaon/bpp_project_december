#ifndef OBJECTTRANSFORMATIONDATA_H
#define OBJECTTRANSFORMATIONDATA_H
#include "SharedHeader.h"
#include "PointCloudTransformationExtraction.h"

class ObjectTransformationData
{
public:
    ObjectTransformationData();

	PointCloudXYZRGB::Ptr object_pointcloud;
	PointCloudTransformationExtraction *transform;
	
	PointTypeXYZRGB target_orientation;
	PointTypeXYZRGB target_position;
	PointTypeXYZRGB input_dimension;

	void CalculateWDH();
	int x_length_mm, y_length_mm, z_length_mm;
	double x_length, y_length, z_length;

	double r, g, b;
	int rotation_case;
};

#endif // OBJECTTRANSFORMATIONDATA_H