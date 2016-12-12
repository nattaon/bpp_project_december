#include "PointCloudOperation.h"

PointCloudOperation::PointCloudOperation()
{
	cout << "PointCloudOperation()" << endl;
	
	planeseg = new PlaneSegmentation();
	clusterextract = new ClusterExtraction();

	container = new ObjectTransformationData();
}

PointCloudOperation::~PointCloudOperation()
{
	delete planeseg;
	delete clusterextract;
}

void PointCloudOperation::SeparateContainerAndItems(vector<PointCloudXYZRGB::Ptr> extract_cluster_cloud)
{
	cout << "SeparateContainerAndItems cluster_cloud size = " << extract_cluster_cloud.size() << endl;

	
	//find cluster which is container
	int max_point_size = 0;
	int max_point_size_id = -1;
	for (int i = 0; i < extract_cluster_cloud.size(); i++)
	{
		if (extract_cluster_cloud[i]->points.size() > max_point_size)
		{
			max_point_size = extract_cluster_cloud[i]->points.size();
			max_point_size_id = i;
		}

	}

	//cout << "max_point_size_id = " << max_point_size_id << "/ size=" << max_point_size << endl;

	//separate container-items
	for (int i = 0; i < extract_cluster_cloud.size(); i++)
	{
		if (i == max_point_size_id)
		{	
			pcl::copyPointCloud(*extract_cluster_cloud[max_point_size_id], *container->object_pointcloud);
		}
		else
		{
			ObjectTransformationData *item = new ObjectTransformationData();
			pcl::copyPointCloud(*extract_cluster_cloud[i], *item->object_pointcloud);
			items.push_back(*item);
		}

	}

	//cout << "items size" << items.size() << endl;


}

void PointCloudOperation::CalculateContainerTransformation()
{
	cout << "CalculateContainerTransformation" << endl;

	container->transform->CalculateTransformation(container->object_pointcloud);
	container->transform->PrintTransformationData();
}
void PointCloudOperation::CalculateItemsTransformation()
{

}

void PointCloudOperation::CalculateTransformation()
{

}