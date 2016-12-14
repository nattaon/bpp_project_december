#include "ObjectTransformationData.h"

ObjectTransformationData::ObjectTransformationData()
{
	object_pointcloud.reset(new PointCloudXYZRGB);
	transform = new PointCloudTransformationExtraction();
}


void ObjectTransformationData::CalculateWDH()
{
	width = (transform->max3d_point.x - transform->min3d_point.x) * 1000;
	depth = (transform->max3d_point.z - transform->min3d_point.z) * 1000;
	height = (transform->max3d_point.y) * 1000;
	//height = (transform->mass_center_point.y) * 1000;
}
