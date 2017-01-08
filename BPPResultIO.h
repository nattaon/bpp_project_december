#ifndef BPPRESULTIO_H
#define BPPRESULTIO_H
#include "SharedHeader.h"

class BPPResultIO
{
public:
    BPPResultIO();

	void WriteBinPackingResult(
		string filename, int total, 
		vector<int> packing_order, vector<int> item_index, vector<int> rotation_case,
		vector<PointTypeXYZRGB> target_position, vector<PointTypeXYZRGB> target_orientation);

	int ReadBinPackingResult(string filename);


	int total_order;
	vector<int> packing_order, item_index, rotation_case;
	vector<PointTypeXYZRGB> target_position, target_orientation;
};

#endif // BPPRESULTIO_H