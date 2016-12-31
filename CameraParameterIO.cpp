#include "CameraParameterIO.h"

CameraParameterIO::CameraParameterIO()
{

}


void CameraParameterIO::WriteCameraParameter(
	string filename,
	double focal_x, double focal_y, double focal_z,
	double pos_x, double pos_y, double pos_z,
	double up_x, double up_y, double up_z,
	double clipping_near, double clipping_far,
	double cam_angle_rad, double cam_angle_deg)
{
	ofstream outfile;
	outfile.open(filename);

	//print : max min
	outfile
		<< focal_x << " "
		<< focal_y << " "
		<< focal_z << endl

		<< pos_x << " "
		<< pos_y << " "
		<< pos_z << endl

		<< up_x << " "
		<< up_y << " "
		<< up_z << endl

		<< clipping_near << " "
		<< clipping_far << endl

		<< cam_angle_rad << " "
		<< cam_angle_deg << endl;

	outfile.close();
}


//pass by reference, to return change of value
void CameraParameterIO::ReadCameraParameter(
	string filename,
	double &focal_x, double &focal_y, double &focal_z,
	double &pos_x, double &pos_y, double &pos_z,
	double &up_x, double &up_y, double &up_z,
	double &clipping_near, double &clipping_far,
	double &cam_angle_rad, double &cam_angle_deg)
{
	ifstream infile;
	infile.open(filename);

	if (!infile.is_open())
	{
		cout << "cannot open file" << endl;
		return;
	}

	string sLine;
	size_t index1 = 0; //unsign int
	size_t index2 = 0;

	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	focal_x = stod(sLine.substr(0, index1));
	focal_y = stod(sLine.substr(index1, index2));
	focal_z = stod(sLine.substr(index2));

	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	pos_x = stod(sLine.substr(0, index1));
	pos_y = stod(sLine.substr(index1, index2));
	pos_z = stod(sLine.substr(index2));

	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	up_x = stod(sLine.substr(0, index1));
	up_y = stod(sLine.substr(index1, index2));
	up_z = stod(sLine.substr(index2));

	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	clipping_near = stod(sLine.substr(0, index1));
	clipping_far = stod(sLine.substr(index1));

	getline(infile, sLine);
	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");
	cam_angle_rad = stod(sLine.substr(0, index1));
	cam_angle_deg = stod(sLine.substr(index1));


	infile.close();
}