#ifndef PLANESEGMENTATION_H
#define PLANESEGMENTATION_H
#include "SharedHeader.h"

class PlaneSegmentation
{
public:
	PlaneSegmentation();
	~PlaneSegmentation();


	void ApplyPlaneSegmentation(double plane_threshold, PointCloudXYZRGB::Ptr cloud);
	void RemovePlane(PointCloudXYZRGB::Ptr cloud);
	void RemovePlaneOutside(PointCloudXYZRGB::Ptr cloud);

private:
	pcl::PointIndices::Ptr plane_inliers;
	pcl::ModelCoefficients::Ptr plane_coefficients;

protected:
	PointCloudXYZRGB::Ptr before_applyplane_cloud;
	PointCloudXYZRGB::Ptr applied_redplane_cloud;
	PointCloudXYZRGB::Ptr removed_plane_cloud;

};

#endif
