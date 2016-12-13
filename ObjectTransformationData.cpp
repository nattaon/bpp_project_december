#include "ObjectTransformationData.h"

ObjectTransformationData::ObjectTransformationData()
{
	object_pointcloud.reset(new PointCloudXYZRGB);
	transform = new PointCloudTransformationExtraction();
}


void ObjectTransformationData::CalculateWDH()
{
	width = (transform->max_point_OBB.x - transform->min_point_OBB.x)*1000;
	depth = (transform->max_point_OBB.y - transform->min_point_OBB.y)*1000;
	height = (transform->max_point_OBB.z - transform->min_point_OBB.z*1000);
}
