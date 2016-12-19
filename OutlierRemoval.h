#ifndef OUTLIERREMOVAL_H
#define OUTLIERREMOVAL_H
#include "SharedHeader.h"

class OutlierRemoval
{
public:
    OutlierRemoval();
	void StatisticalOutlierRemoval(
		PointCloudXYZRGB::Ptr cloud, int meank, double stddev);
};

#endif // OUTLIERREMOVAL_H