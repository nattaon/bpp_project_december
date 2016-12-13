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
	int width, depth, height;
	Eigen::Vector3f target_orientation;
	PointTypeXYZRGB target_position;

	void CalculateWDH();
};

#endif // OBJECTTRANSFORMATIONDATA_H