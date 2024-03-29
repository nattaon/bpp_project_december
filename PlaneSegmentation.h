#ifndef PLANESEGMENTATION_H
#define PLANESEGMENTATION_H
#include "SharedHeader.h"
#include "PointCloudTransformationExtraction.h"
#include "PassthroughFilter.h"

class PlaneSegmentation:public PassthroughFilter
{
public:
	PlaneSegmentation();
	~PlaneSegmentation();


	void SetHasPlaneTransformData(bool value);
	bool isPlaneTransformDataAvailable();

	PointCloudTransformationExtraction *transformextract;

	void ApplyPlaneSegmentation(double plane_threshold, PointCloudXYZRGB::Ptr cloud);
	void RemovePlane(PointCloudXYZRGB::Ptr cloud);
	void CalculatePlaneTransformation(double plane_threshold, PointCloudXYZRGB::Ptr cloud);
	void RemovePlaneOutside(PointCloudXYZRGB::Ptr cloud);
	void RemovePlaneOutsideAfterAxisAlign(PointCloudXYZRGB::Ptr cloud);

	PointCloudXYZRGB::Ptr before_applyplane_cloud;
	PointCloudXYZRGB::Ptr applied_redplane_cloud;
	PointCloudXYZRGB::Ptr removed_plane_cloud;
	PointCloudXYZRGB::Ptr removed_planeoutside_cloud;
	PointCloudXYZRGB::Ptr only_plane_cloud;

private:
	pcl::PointIndices::Ptr plane_inliers;
	pcl::ModelCoefficients::Ptr plane_coefficients;

	bool isHasPlaneTransformData;

protected:

};

#endif
