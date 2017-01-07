#ifndef POINTCLOUDBPPTEXTLISTIO_H
#define POINTCLOUDBPPTEXTLISTIO_H
#include "SharedHeader.h"

class PointCloudBPPTextListIO
{
public:
	PointCloudBPPTextListIO();

	//not pass as ObjectTransformationData because
	//need to read fresh value from ui (in case of user edit value)
	void WritePointCloudListForBPP(
		string filename, int total,
		int bin_x_dim, int bin_y_dim, int bin_z_dim,
		PointTypeXYZRGB bin_min_pos, PointTypeXYZRGB bin_max_pos,
		Eigen::Vector3f bin_major_vector, Eigen::Vector3f bin_middle_vector, Eigen::Vector3f bin_minor_vector,
		vector<string> array_pcd_filename,
		vector<int> items_x_dim, vector<int> items_y_dim, vector<int> items_z_dim,
		vector<PointTypeXYZRGB> items_min_pos, vector<PointTypeXYZRGB> items_max_pos,
		vector<Eigen::Vector3f> items_major_vector, vector<Eigen::Vector3f> items_middle_vector, vector<Eigen::Vector3f> items_minor_vector
		);

	int ReadPointCloudListForBPP(string filename);
	/*, int &total);
		int &bin_width, int &bin_height, int &bin_depth,
		vector<string> &array_pcd_filename,
		int *item_w, int *item_h, int *item_d);*/

	

	int total_items;
	
	int bin_x_dim, bin_y_dim, bin_z_dim;
	PointTypeXYZRGB bin_min_pos, bin_max_pos;
	Eigen::Vector3f bin_major_vector,  bin_middle_vector, bin_minor_vector;
	
	vector<string> array_pcd_filename;
	
	vector<int> items_x_dim, items_y_dim, items_z_dim;
	vector<PointTypeXYZRGB> items_min_pos, items_max_pos;
	vector<Eigen::Vector3f> items_major_vector, items_middle_vector, items_minor_vector;


};

#endif // POINTCLOUDBPPTEXTLISTIO_H