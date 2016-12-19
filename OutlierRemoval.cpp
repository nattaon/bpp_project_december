#include "OutlierRemoval.h"

OutlierRemoval::OutlierRemoval()
{

}

void OutlierRemoval::StatisticalOutlierRemoval(
	PointCloudXYZRGB::Ptr cloud, int meank, double stddev)
{
	cout << "before outlier remove cloud->size() " << cloud->size() << endl;

	//int meank = cloud->points.size()*meank_percentage*0.01;
	pcl::StatisticalOutlierRemoval<PointTypeXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(meank);//The number of neighbors to analyze for each point eg 50
	sor.setStddevMulThresh(stddev);//the standard deviation multiplier

	PointCloudXYZRGB::Ptr cloud_filtered(new PointCloudXYZRGB);
	sor.filter(*cloud_filtered);
	if (cloud_filtered->size() != 0)
	{
		pcl::copyPointCloud(*cloud_filtered, *cloud);
	}
	cout << "after outlier remove cloud->size() " << cloud->size() << endl;

}