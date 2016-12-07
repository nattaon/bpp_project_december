#include "PointCloudTransformationExtraction.h"

PointCloudTransformationExtraction::PointCloudTransformationExtraction()
{

}

void PointCloudTransformationExtraction::CalculateTransformation(PointCloudXYZRGB::Ptr cloud)
{
	//need to filter out data for speed calculation

	//VoxelGridFilter *voxel;
	//voxel = new VoxelGridFilter();

	cout << "before 0.05 voxel : cloud size = " << cloud->size() << endl;
	//voxel->
	FilterVoxelSize(cloud, 0.05);//distance between point 0.05 = 5cm
	cout << "after 0.05 voxel : cloud size = " << cloud->size() << endl;

	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getMassCenter(mass_center);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

	//delete voxel;

}

void PointCloudTransformationExtraction::CalculateWDH()
{

}

