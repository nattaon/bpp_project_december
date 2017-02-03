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
void ClusterExtraction::ExtractCluster(PointCloudXYZRGB::Ptr cloud)
{
	
	cout << "ShowClusterInColor" << endl;
	cout << "cloud size = " << cloud->size() << endl;
	cout << "min cluster size =" << min_size << endl;
	cout << "max cluster size =" << max_size << endl;

	color_cluster_cloud.reset(new PointCloudXYZRGB);
	//pcl::copyPointCloud(*cloud, *color_cluster_cloud);

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

	QTime timer;
	timer.start();
	ec.extract(vector_cluster_indices);	
	int nMilliseconds = timer.elapsed();
	cout << "ec.extract timer elapsed " << nMilliseconds << " msec" << endl;
	cout << "cluster total =  " << vector_cluster_indices.size() << endl;
	
	array_cluster_cloud.clear();
	cout << "array_cluster_cloud size" << array_cluster_cloud.size() << endl;
	int cluster_all_size = 0;

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
			cluster_all_size++;
			color_cluster_cloud->points.push_back(cloud->points[*pit]);
			//this copy new point, not just a pointer
			
			color_cluster_cloud->points[color_cluster_cloud->size()-1].rgb = color;
		}
		array_cluster_cloud.push_back(cluster_cloud);

	}
	cout << "array_cluster_cloud size " << array_cluster_cloud.size() << endl;
	cout << "cluster_all_size size " << cluster_all_size << endl;
	cout << "color_cluster_cloud size " << color_cluster_cloud->points.size() << endl;
}

std::vector<PointCloudXYZRGB::Ptr> ClusterExtraction::GetExtractCluster()
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
	//cout << r << "," << g << "," << b << endl; // 97,123,190
	//cout << "color = " << rgb << endl; //6388670

	return rgb;
}
