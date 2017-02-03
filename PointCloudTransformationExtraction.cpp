#include "PointCloudTransformationExtraction.h"

PointCloudTransformationExtraction::PointCloudTransformationExtraction()
{

}

void PointCloudTransformationExtraction::CalculateTransformation(
	PointCloudXYZRGB::Ptr cloud, double voxel_size)
{
	//need to filter out data for speed calculation

	//VoxelGridFilter *voxel;
	//voxel = new VoxelGridFilter();

	cout << "before " << voxel_size << " voxel : cloud size = " << cloud->size() << endl;
	//voxel->
	if (voxel_size!=0.0)
	{ 
		FilterVoxelSize(cloud, voxel_size);//distance between point 0.05 = 5cm
		cout << "after " << voxel_size << " voxel : cloud size = " << cloud->size() << endl;
	}

	feature_extractor.setInputCloud(cloud);
	QTime timer;
	timer.start();
		feature_extractor.compute();
	int nMilliseconds = timer.elapsed();
	cout << "feature_extractor.compute() timer elapsed " << nMilliseconds << " msec" << endl;
	feature_extractor.getMassCenter(mass_center);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

	//delete voxel;

	mass_center_point.x = mass_center[0];
	mass_center_point.y = mass_center[1];
	mass_center_point.z = mass_center[2];

	CalculateMinMaxPoint(cloud);



}

void PointCloudTransformationExtraction::CalculateMinMaxPoint(
	PointCloudXYZRGB::Ptr cloud)
{
	pcl::getMinMax3D(*cloud, min3d_point, max3d_point);
}


void PointCloudTransformationExtraction::PrintTransformationData()
{
	cout << "mass_center is \n" << mass_center << endl;
	cout << "major_vector is \n" << major_vector << endl;
	cout << "middle_vector is \n" << middle_vector << endl;
	cout << "minor_vector is \n" << minor_vector << endl;
	cout << "min_point_OBB is \n" << min_point_OBB << endl;
	cout << "max_point_OBB is \n" << max_point_OBB << endl;
	cout << "position_OBB is \n" << position_OBB << endl;
	cout << "rotational_matrix_OBB is \n" << rotational_matrix_OBB << endl;
	cout << "min3d_point is \n" << min3d_point << endl;
	cout << "max3d_point is \n" << max3d_point << endl;
}


