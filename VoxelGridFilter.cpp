#include "VoxelGridFilter.h"


VoxelGridFilter::VoxelGridFilter()
{
}


VoxelGridFilter::~VoxelGridFilter()
{
}

void VoxelGridFilter::FilterVoxelSize(PointCloudXYZRGB::Ptr cloud, double size)
{

	pcl::VoxelGrid<PointTypeXYZRGB> grid;

	//set scope for filter
	grid.setLeafSize(size, size, size);

	//set cloud to filter
	grid.setInputCloud(cloud);

	//do filter(save output in new pointcloud)
	PointCloudXYZRGB::Ptr cloud_filtered(new PointCloudXYZRGB);

	QTime timer;
	timer.start();
		grid.filter(*cloud_filtered);
	int nMilliseconds = timer.elapsed();
	cout << "grid.filter timer elapsed " << nMilliseconds << " msec" << endl;


	//return pointcloud
	if (cloud_filtered->size() != 0)
	{
		pcl::copyPointCloud(*cloud_filtered, *cloud);
	}

	//show number of pointclound after apply filter
	//pcl::console::print_info( "after point clouds : %d\n", cloud->size() );
}
