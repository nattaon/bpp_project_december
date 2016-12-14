#ifndef CAMERAPARAMETERIO_H
#define CAMERAPARAMETERIO_H
#include "SharedHeader.h"

class CameraParameterIO
{
public:
    CameraParameterIO();

	void WriteCameraParameter(
		string filename,
		double focal_x, double focal_y, double focal_z,
		double pos_x, double pos_y, double pos_z,
		double up_x, double up_y, double up_z,
		double clipping_near, double clipping_far,
		double cam_angle_rad, double cam_angle_deg);


	//pass by reference, to return change of value
	void ReadCameraParameter(
		string filename,
		double &focal_x, double &focal_y, double &focal_z,
		double &pos_x, double &pos_y, double &pos_z,
		double &up_x, double &up_y, double &up_z,
		double &clipping_near, double &clipping_far,
		double &cam_angle_rad, double &cam_angle_deg);



};

#endif // CAMERAPARAMETERIO_H