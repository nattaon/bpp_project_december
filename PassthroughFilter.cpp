#include "PassthroughFilter.h"

PassthroughFilter::PassthroughFilter()
{

}

PassthroughFilter::~PassthroughFilter()
{

}



void PassthroughFilter::Filter(PointCloudXYZRGB::Ptr cloud, std::string axis, float min, float max)
{

	cout << "filter " << axis << ":" << min << "," << max << endl;
	if (max == 99 || min == 99)
	{
		//cout << "nofilter" << endl;
		return;
	}
	PointCloudXYZRGB::Ptr cloud_filtered(new PointCloudXYZRGB);

	pcl::PassThrough<PointTypeXYZRGB> pass;

	pass.setInputCloud(cloud);
	pass.setFilterFieldName(axis);
	pass.setFilterLimits(min, max);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filtered);
	//pcl::console::print_info("cloud_filtered sized : %d\n", cloud_filtered->size());

	// cloud_filtered->size()==0 will cause error "vector subscript out of range"
	if (cloud_filtered->size() != 0)
	{
		pcl::copyPointCloud(*cloud_filtered, *cloud);
	}
	else
	{
		//cout << "cloud_filtered->size()==0 " << endl;

	}
	
}
