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

	int ReadPointCloudListForBPP(string filename);
	/*, int &total);
		int &bin_width, int &bin_height, int &bin_depth,
		vector<string> &array_pcd_filename,
		int *item_w, int *item_h, int *item_d);*/

	int total_items;
	int bin_width, bin_height, bin_depth;
	vector<string> array_pcd_filename;
	int *item_w, *item_h, *item_d;

};

#endif // POINTCLOUDBPPTEXTLISTIO_H