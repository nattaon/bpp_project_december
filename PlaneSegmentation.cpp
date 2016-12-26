#include "PlaneSegmentation.h"


PlaneSegmentation::PlaneSegmentation()
{
	SetHasPlaneTransformData(false);

	transformextract = new PointCloudTransformationExtraction();

	before_applyplane_cloud.reset(new PointCloudXYZRGB);
	applied_redplane_cloud.reset(new PointCloudXYZRGB);
	removed_plane_cloud.reset(new PointCloudXYZRGB);
	removed_planeoutside_cloud.reset(new PointCloudXYZRGB);
	only_plane_cloud.reset(new PointCloudXYZRGB);
}


PlaneSegmentation::~PlaneSegmentation()
{
	delete transformextract;
}
void PlaneSegmentation::SetHasPlaneTransformData(bool value)
{
	isHasPlaneTransformData = value;
}

bool PlaneSegmentation::isPlaneTransformDataAvailable()
{
	return isHasPlaneTransformData;
}


void PlaneSegmentation::ApplyPlaneSegmentation(double plane_threshold, PointCloudXYZRGB::Ptr cloud)
{
	pcl::copyPointCloud(*cloud, *before_applyplane_cloud);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// 1. find pointclouds index(inlier) which is locate in plane
	pcl::SACSegmentation<PointTypeXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(plane_threshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	plane_inliers = inliers;
	plane_coefficients = coefficients;

	transformextract->plane_coefficients_matrix(0) = plane_coefficients->values[0];
	transformextract->plane_coefficients_matrix(1) = plane_coefficients->values[1];
	transformextract->plane_coefficients_matrix(2) = plane_coefficients->values[2];

	//cout << "coefficients : " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << endl;


	// 2. extract plane and not plane pointcloud from inlier
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(plane_inliers);

	pcl::copyPointCloud(*cloud, *removed_plane_cloud);
	pcl::copyPointCloud(*cloud, *only_plane_cloud);

	//true:remove plane, flase:remove not plane
	extract.setNegative(true);
	extract.filter(*removed_plane_cloud);

	extract.setNegative(false);
	extract.filter(*only_plane_cloud);

	int r = rand() % 128+ 128;
	cout << "r= " << r << endl;

	// 3. color plane as red to input pointcloud
	for (int i = 0; i < inliers->indices.size(); i++){
		cloud->points[inliers->indices[i]].r = r;
		cloud->points[inliers->indices[i]].g = 0;
		cloud->points[inliers->indices[i]].b = 0;
	}

	pcl::copyPointCloud(*cloud, *applied_redplane_cloud);

}
void PlaneSegmentation::RemovePlane(PointCloudXYZRGB::Ptr cloud)
{
	/*
	if (!plane_inliers)
	{
		QMessageBox::information(0, QString("Remove plane"), QString("No plane to remove, Segment plane first"), QMessageBox::Ok);
		return;
	}

	PointCloudXYZRGB::Ptr tmp(new PointCloudXYZRGB);
	pcl::copyPointCloud(*cloud, *tmp);

	//filtering
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	//extract.setInputCloud(tmp);
	extract.setInputCloud(cloud);
	extract.setIndices(plane_inliers);

	pcl::copyPointCloud(*cloud, *removed_plane_cloud);
	pcl::copyPointCloud(*cloud, *only_plane_cloud);
	//true:remove plane, flase:remove not plane
	extract.setNegative(true);
	extract.filter(*removed_plane_cloud);

	extract.setNegative(false);
	extract.filter(*only_plane_cloud);
*/
}

void PlaneSegmentation::CalculatePlaneTransformation(PointCloudXYZRGB::Ptr cloud)
{
	transformextract->CalculateTransformation(cloud,0.05); //only_plane_cloud

	SetHasPlaneTransformData(true);

	transformextract->PrintTransformationData();


}


void PlaneSegmentation::RemovePlaneOutside(PointCloudXYZRGB::Ptr cloud)
{
	if (!isPlaneTransformDataAvailable())
	{
		QMessageBox::information(0, QString("Remove plane outside"), QString("No plane transformation data. Please segment plane or load plane first"), QMessageBox::Ok);
		return;
	}
	if (cloud->points.size()==0)
	{
		QMessageBox::information(0, QString("Remove plane outside"), QString("No pointcloud"), QMessageBox::Ok);
		return;

	}

	Filter(cloud, "x", transformextract->min3d_point.x, transformextract->max3d_point.x);
	Filter(cloud, "y", transformextract->min3d_point.y, transformextract->max3d_point.y);
	Filter(cloud, "z", transformextract->min3d_point.z-0.5, transformextract->max3d_point.z);

	pcl::copyPointCloud(*cloud, *removed_planeoutside_cloud);




}

void PlaneSegmentation::RemovePlaneOutsideAfterAxisAlign(PointCloudXYZRGB::Ptr cloud)
{
	if (!isPlaneTransformDataAvailable())
	{
		QMessageBox::information(0, QString("Remove plane outside"), QString("No plane transformation data. Please segment plane or load plane first"), QMessageBox::Ok);
		return;
	}
	if (cloud->points.size() == 0)
	{
		QMessageBox::information(0, QString("Remove plane outside"), QString("No pointcloud"), QMessageBox::Ok);
		return;

	}	
	//calculate min max point of cloud
	transformextract->CalculateMinMaxPoint(cloud);

	Filter(cloud, "x", transformextract->min3d_point.x, transformextract->max3d_point.x);
	Filter(cloud, "y", transformextract->min3d_point.y, transformextract->max3d_point.y);
	Filter(cloud, "z", transformextract->min3d_point.z - 0.5, transformextract->max3d_point.z + 1.5);

	pcl::copyPointCloud(*cloud, *removed_planeoutside_cloud);

}

