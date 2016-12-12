#include "PlaneSegmentParameterIO.h"

PlaneSegmentParameterIO::PlaneSegmentParameterIO()
{

}

void PlaneSegmentParameterIO::WritePlaneTransformParameter(
    string filename,
	Eigen::Matrix3f plane_coefficients_matrix,
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

	

	outfile
		//writing plane_coefficients variable is not easy (need header value)
		<< "//plane_coefficients" << endl
		<< plane_coefficients_matrix(0) << " "
		<< plane_coefficients_matrix(1) << " "
		<< plane_coefficients_matrix(2) << endl
		<< "//" << endl

		<< "//mass_center" << endl
		<< mass_center(0) << " "
		<< mass_center(1) << " "
		<< mass_center(2) << endl
		<< "//" << endl

		<< "//major_vector" << endl
		<< major_vector(0) << " "
		<< major_vector(1) << " "
		<< major_vector(2) << endl
		<< "//" << endl

		<< "//middle_vector" << endl
		<< middle_vector(0) << " "
		<< middle_vector(1) << " "
		<< middle_vector(2) << endl
		<< "//" << endl

		<< "//minor_vector" << endl
		<< minor_vector(0) << " "
		<< minor_vector(1) << " "
		<< minor_vector(2) << endl
		<< "//" << endl

		<< "//min_point_OBB" << endl
		<< min_point_OBB.x << " "
		<< min_point_OBB.y << " "
		<< min_point_OBB.z << endl
		<< "//" << endl

		<< "//max_point_OBB" << endl
		<< max_point_OBB.x << " "
		<< max_point_OBB.y << " "
		<< max_point_OBB.z << endl
		<< "//" << endl

		<< "//position_OBB" << endl
		<< position_OBB.x << " "
		<< position_OBB.y << " "
		<< position_OBB.z << endl
		<< "//" << endl

		<< "//rotational_matrix_OBB" << endl
		<< rotational_matrix_OBB(0) << " "
		<< rotational_matrix_OBB(1) << " "
		<< rotational_matrix_OBB(2) << endl 
		<< rotational_matrix_OBB(3) << " "
		<< rotational_matrix_OBB(4) << " "
		<< rotational_matrix_OBB(5) << endl 
		<< rotational_matrix_OBB(6) << " "
		<< rotational_matrix_OBB(7) << " "
		<< rotational_matrix_OBB(8) << endl
		<< "//" << endl;

    outfile.close();

}

//pass by reference, to return change of value
void PlaneSegmentParameterIO::ReadPlaneTransformParameter(
    string filename,
	Eigen::Matrix3f &plane_coefficients_matrix,
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

	string sLine;
	int index1 = 0;
	int index2 = 0;

	//writing plane_coefficients variable is not easy (need header value)
	getline(infile, sLine); //plane_coefficients
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	plane_coefficients_matrix(0) = stod(sLine.substr(0, index1));
	plane_coefficients_matrix(1) = stod(sLine.substr(index1, index2));
	plane_coefficients_matrix(2) = stod(sLine.substr(index2));
	getline(infile, sLine);//

	

	getline(infile, sLine); //mass_center
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	mass_center(0) = stod(sLine.substr(0, index1));
	mass_center(1) = stod(sLine.substr(index1, index2));
	mass_center(2) = stod(sLine.substr(index2));
	getline(infile, sLine);//

	getline(infile, sLine); //major_vector
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	major_vector(0) = stod(sLine.substr(0, index1));
	major_vector(1) = stod(sLine.substr(index1, index2));
	major_vector(2) = stod(sLine.substr(index2));
	getline(infile, sLine);//

	getline(infile, sLine); //middle_vector
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	middle_vector(0) = stod(sLine.substr(0, index1));
	middle_vector(1) = stod(sLine.substr(index1, index2));
	middle_vector(2) = stod(sLine.substr(index2));
	getline(infile, sLine);//

	getline(infile, sLine); //minor_vector
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	minor_vector(0) = stod(sLine.substr(0, index1));
	minor_vector(1) = stod(sLine.substr(index1, index2));
	minor_vector(2) = stod(sLine.substr(index2));
	getline(infile, sLine);//

	getline(infile, sLine); //min_point_OBB
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	min_point_OBB.x = stod(sLine.substr(0, index1));
	min_point_OBB.y = stod(sLine.substr(index1, index2));
	min_point_OBB.z = stod(sLine.substr(index2));
	getline(infile, sLine);//

	getline(infile, sLine); //max_point_OBB
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	max_point_OBB.x = stod(sLine.substr(0, index1));
	max_point_OBB.y = stod(sLine.substr(index1, index2));
	max_point_OBB.z = stod(sLine.substr(index2));
	getline(infile, sLine);//	
	
	getline(infile, sLine); //position_OBB
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	position_OBB.x = stod(sLine.substr(0, index1));
	position_OBB.y = stod(sLine.substr(index1, index2));
	position_OBB.z = stod(sLine.substr(index2));
	getline(infile, sLine);//

	getline(infile, sLine); //rotational_matrix_OBB
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	rotational_matrix_OBB(0) = stod(sLine.substr(0, index1));
	rotational_matrix_OBB(1) = stod(sLine.substr(index1, index2));
	rotational_matrix_OBB(2) = stod(sLine.substr(index2));
	
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	rotational_matrix_OBB(3) = stod(sLine.substr(0, index1));
	rotational_matrix_OBB(4) = stod(sLine.substr(index1, index2));
	rotational_matrix_OBB(5) = stod(sLine.substr(index2));
	
	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	rotational_matrix_OBB(6) = stod(sLine.substr(0, index1));
	rotational_matrix_OBB(7) = stod(sLine.substr(index1, index2));
	rotational_matrix_OBB(8) = stod(sLine.substr(index2));
	getline(infile, sLine);//


    infile.close();

}
