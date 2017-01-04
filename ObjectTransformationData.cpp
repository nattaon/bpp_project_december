#include "ObjectTransformationData.h"

ObjectTransformationData::ObjectTransformationData()
{
	object_pointcloud.reset(new PointCloudXYZRGB);
	transform = new PointCloudTransformationExtraction();
}


void ObjectTransformationData::CalculateWDH()
{
	x_length = (transform->max3d_point.x - transform->min3d_point.x);
	y_length = transform->max3d_point.y;
	z_length = (transform->max3d_point.z - transform->min3d_point.z);

	x_length_mm = x_length * 1000;
	y_length_mm = y_length * 1000;
	z_length_mm = z_length * 1000;
	
	input_dimension.x = x_length;
	input_dimension.y = y_length;
	input_dimension.z = z_length;


}
