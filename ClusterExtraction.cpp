#include "ClusterExtraction.h"


ClusterExtraction::ClusterExtraction()
{
}


ClusterExtraction::~ClusterExtraction()
{
}

void ClusterExtraction::SetClusterExtractValue(
	double cluster_tolerance, int cluster_min_size, int cluster_max_size)
{
	tolerance = cluster_tolerance;
	min_size = cluster_min_size;
	max_size = cluster_max_size;
}
void ClusterExtraction::ShowClusterInColor(PointCloudXYZRGB::Ptr cloud)
{
	cout << "ShowClusterInColor" << endl;
	cout << "cloud size = " << cloud->size() << endl;
	cout << "min cluster size =" << min_size << endl;
	cout << "max cluster size =" << max_size << endl;

	applied_colorcluster_cloud.reset(new PointCloudXYZRGB);
	pcl::copyPointCloud(*cloud, *applied_colorcluster_cloud);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointTypeXYZRGB>::Ptr tree(new pcl::search::KdTree<PointTypeXYZRGB>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> vector_cluster_indices;
	vector_cluster_indices.clear();

	pcl::EuclideanClusterExtraction<PointTypeXYZRGB> ec;
	ec.setClusterTolerance(tolerance); //0.02=2cm   //0.005
	ec.setMinClusterSize(min_size);
	ec.setMaxClusterSize(max_size);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);

	ec.extract(vector_cluster_indices);	
	cout << "cluster total =  " << vector_cluster_indices.size() << endl;

	array_cluster_cloud.clear();
	cout << "array_cluster_cloud size" << array_cluster_cloud.size() << endl;

	for (vector<pcl::PointIndices>::const_iterator it = vector_cluster_indices.begin(); it != vector_cluster_indices.end(); ++it)
	{

		PointCloudXYZRGB::Ptr cluster_cloud(new PointCloudXYZRGB);
		int color = randomcolor();
		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			//cloud->points[*pit].rgb = color;
			//do blend color between 2 rgb to get transparent color

			//put original object color to array
			cluster_cloud->points.push_back(cloud->points[*pit]);
			applied_colorcluster_cloud->points[*pit].rgb = color;
		}
		array_cluster_cloud.push_back(cluster_cloud);

	}
	cout << "array_cluster_cloud size" << array_cluster_cloud.size() << endl;
}

std::vector<PointCloudXYZRGB::Ptr> ClusterExtraction::ExtractCluster()
{
	return array_cluster_cloud;
}

int ClusterExtraction::randomcolor()
{
	int r = rand() % 200 + 56;
	int g = rand() % 200 + 56;
	int b = rand() % 200 + 56;

	int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);

	//cout << "\n" << endl;
	//cout << r << "," << g << "," << b << endl;
	cout << "color = " << rgb << endl;

	return rgb;
}
