#ifndef VOXELGRIDFILTER_H
#define VOXELGRIDFILTER_H
#include "SharedHeader.h"

class VoxelGridFilter
{
public:
	VoxelGridFilter();
	~VoxelGridFilter();

	void FilterVoxelSize(PointCloudXYZRGB::Ptr cloud, double size);

};

#endif
