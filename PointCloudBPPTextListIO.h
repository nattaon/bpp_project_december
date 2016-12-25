#ifndef POINTCLOUDBPPTEXTLISTIO_H
#define POINTCLOUDBPPTEXTLISTIO_H
#include "SharedHeader.h"

class PointCloudBPPTextListIO
{
public:
	PointCloudBPPTextListIO();

	void WritePointCloudListForBPP(
		string filename, int total,
		int bin_width, int bin_height, int bin_depth,
		vector<string> array_pcd_filename,
		int *item_w, int *item_h, int *item_d);

	void ReadPointCloudListForBPP(
		string filename, int &total,
		int &bin_width, int &bin_height, int &bin_depth,
		vector<string> &array_pcd_filename,
		int *item_w, int *item_h, int *item_d);

};

#endif // POINTCLOUDBPPTEXTLISTIO_H