#include "PlaneSegmentParameterIO.h"

PlaneSegmentParameterIO::PlaneSegmentParameterIO()
{

}

void PlaneSegmentParameterIO::WritePlaneTransformParameter(
    string filename,
	pcl::ModelCoefficients::Ptr plane_coefficients,
    Eigen::Vector3f mass_center,
    Eigen::Vector3f major_vector,
    Eigen::Vector3f middle_vector,
    Eigen::Vector3f minor_vector,
    PointTypeXYZRGB min_point_OBB,
    PointTypeXYZRGB max_point_OBB,
    PointTypeXYZRGB position_OBB,
    Eigen::Matrix3f rotational_matrix_OBB)
{
    ofstream outfile;
    outfile.open(filename);

	

  /*  outfile
        << passthrough_xmax << " "
        << passthrough_xmin << endl

        << passthrough_ymax << " "
        << passthrough_ymin << endl

        << passthrough_zmax << " "
        << passthrough_zmin << endl;
*/
    outfile.close();

}

//pass by reference, to return change of value
void PlaneSegmentParameterIO::ReadPlaneTransformParameter(
    string filename,
    pcl::ModelCoefficients::Ptr &plane_coefficients,
    Eigen::Vector3f &mass_center,
    Eigen::Vector3f &major_vector,
    Eigen::Vector3f &middle_vector,
    Eigen::Vector3f &minor_vector,
    PointTypeXYZRGB &min_point_OBB,
    PointTypeXYZRGB &max_point_OBB,
    PointTypeXYZRGB &position_OBB,
    Eigen::Matrix3f &rotational_matrix_OBB)
{

    ifstream infile;
    infile.open(filename);

    if (!infile.is_open())
    {
        cout << "cannot open file" << endl;
        return;
    }





    infile.close();

}
