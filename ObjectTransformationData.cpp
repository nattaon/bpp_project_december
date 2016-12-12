#include "ObjectTransformationData.h"

ObjectTransformationData::ObjectTransformationData()
{
	object_pointcloud.reset(new PointCloudXYZRGB);
	transform = new PointCloudTransformationExtraction();
}
