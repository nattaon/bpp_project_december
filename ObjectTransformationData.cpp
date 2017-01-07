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

void ObjectTransformationData::SetLengthMM(int box_x_dim, int box_y_dim, int box_z_dim)
{
	x_length_mm = box_x_dim;
	y_length_mm = box_y_dim;
	z_length_mm = box_z_dim;

	x_length = x_length_mm*0.001;
	y_length = y_length_mm*0.001;
	z_length = z_length_mm*0.001;

	input_dimension.x = x_length;
	input_dimension.y = y_length;
	input_dimension.z = z_length;

}
void ObjectTransformationData::SetTransformMinMaxPos(PointTypeXYZRGB items_min_pos, PointTypeXYZRGB items_max_pos)
{
	transform->min3d_point = items_min_pos;
	transform->max3d_point = items_max_pos;

	transform->mass_center_point.x = (transform->min3d_point.x + transform->max3d_point.x) / 2;
	transform->mass_center_point.y = (transform->min3d_point.y + transform->max3d_point.y) / 2;
	transform->mass_center_point.z = (transform->min3d_point.z + transform->max3d_point.z) / 2;
}
void ObjectTransformationData::SetTransformVector(
	Eigen::Vector3f items_major_vector,
	Eigen::Vector3f items_middle_vector,
	Eigen::Vector3f items_minor_vector)
{
	transform->major_vector = items_major_vector;
	transform->middle_vector = items_middle_vector;
	transform->minor_vector = items_minor_vector;
}