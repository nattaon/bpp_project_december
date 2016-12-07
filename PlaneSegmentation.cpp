#include "PlaneSegmentation.h"


PlaneSegmentation::PlaneSegmentation()
{

	before_applyplane_cloud.reset(new PointCloudXYZRGB);
	applied_redplane_cloud.reset(new PointCloudXYZRGB);
	removed_plane_cloud.reset(new PointCloudXYZRGB);
}


PlaneSegmentation::~PlaneSegmentation()
{

}

void PlaneSegmentation::ApplyPlaneSegmentation(double plane_threshold, PointCloudXYZRGB::Ptr cloud)
{
	pcl::copyPointCloud(*cloud, *before_applyplane_cloud);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<PointTypeXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(plane_threshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);


	//cout << "coefficients : " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << endl;

	
	{
		// color plane to red
		for (size_t i = 0; i < inliers->indices.size(); i++){
			cloud->points[inliers->indices[i]].r = 255;
			cloud->points[inliers->indices[i]].g = 0;
			cloud->points[inliers->indices[i]].b = 0;
		}
	}

	plane_inliers = inliers;
	plane_coefficients = coefficients;

	pcl::copyPointCloud(*cloud, *applied_redplane_cloud);

}
void PlaneSegmentation::RemovePlane(PointCloudXYZRGB::Ptr cloud)
{
	if (!plane_inliers)
	{
		QMessageBox::information(0, QString("Remove plane"), QString("No plane to remove, Segment plane first"), QMessageBox::Ok);
		return;
	}

	PointCloudXYZRGB::Ptr tmp(new PointCloudXYZRGB);
	pcl::copyPointCloud(*cloud, *tmp);

	//filtering
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(tmp);
	extract.setIndices(plane_inliers);

	//true:remove plane, flase:remove not plane
	extract.setNegative(true);
	extract.filter(*cloud);

	pcl::copyPointCloud(*cloud, *removed_plane_cloud);

}
void PlaneSegmentation::RemovePlaneOutside(PointCloudXYZRGB::Ptr cloud)
{

}
