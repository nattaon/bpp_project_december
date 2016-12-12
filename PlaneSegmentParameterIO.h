#ifndef PLANESEGMENTPARAMETERIO_H
#define PLANESEGMENTPARAMETERIO_H
#include "SharedHeader.h"

class PlaneSegmentParameterIO
{
public:
    PlaneSegmentParameterIO();

    void WritePlaneTransformParameter(
        string filename,
		Eigen::Matrix3f plane_coefficients_matrix,
        Eigen::Vector3f mass_center,
        Eigen::Vector3f major_vector,
        Eigen::Vector3f middle_vector,
        Eigen::Vector3f minor_vector,
        PointTypeXYZRGB min_point_OBB,
        PointTypeXYZRGB max_point_OBB,
        PointTypeXYZRGB position_OBB,
        Eigen::Matrix3f rotational_matrix_OBB);

    //pass by reference, to return change of value
	void ReadPlaneTransformParameter(
        string filename,
		Eigen::Matrix3f &plane_coefficients_matrix,
        Eigen::Vector3f &mass_center,
        Eigen::Vector3f &major_vector,
        Eigen::Vector3f &middle_vector,
        Eigen::Vector3f &minor_vector,
        PointTypeXYZRGB &min_point_OBB,
        PointTypeXYZRGB &max_point_OBB,
        PointTypeXYZRGB &position_OBB,
        Eigen::Matrix3f &rotational_matrix_OBB);

};

#endif // PLANESEGMENTPARAMETERIO_H
