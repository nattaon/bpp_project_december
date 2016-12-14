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
	
	Eigen::Vector3f target_orientation;
	PointTypeXYZRGB target_position;

	void CalculateWDH();
	int width, depth, height;

	double r, g, b;
};

#endif // OBJECTTRANSFORMATIONDATA_H