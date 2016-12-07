#ifndef PASSTHROUGHFILTER_H
#define PASSTHROUGHFILTER_H

#include "SharedHeader.h"

class PassthroughFilter
{

public:
	PassthroughFilter();
	~PassthroughFilter();

	void Filter(PointCloudXYZRGB::Ptr cloud, std::string axis, float min, float max);


};

#endif;