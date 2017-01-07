#include "PointCloudOperation.h"

PointCloudOperation::PointCloudOperation()
{
	cout << "PointCloudOperation()" << endl;
	
	planeseg = new PlaneSegmentation();
	clusterextract = new ClusterExtraction();
	outlierremove = new OutlierRemoval();
	voxelfilter = new VoxelGridFilter();

	container = new ObjectTransformationData();

	isSetAlignCorner = false;
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
			max_point_size = static_cast<int>(extract_cluster_cloud[i]->points.size());
			max_point_size_id = i;
		}

	}

	//cout << "max_point_size_id = " << max_point_size_id << "/ size=" << max_point_size << endl;
	ClearVectorItems();
	
	cout << "items size" << items.size() << endl;
	//separate container-items
	for (int i = 0; i < extract_cluster_cloud.size(); i++)
	{
		if (i == max_point_size_id)
		{
			container = new ObjectTransformationData();
			pcl::copyPointCloud(*extract_cluster_cloud[max_point_size_id], *container->object_pointcloud);
		}
		else
		{
			ObjectTransformationData *item = new ObjectTransformationData();
			pcl::copyPointCloud(*extract_cluster_cloud[i], *item->object_pointcloud);
			items.push_back(item);
		}

	}

	//


}
void PointCloudOperation::ClearVectorItems()
{
	cout << "PointCloudOperation::ClearVectorItems()" << endl;
	items.clear();
	cout << "items.size() = " << items.size() << endl;
}
void PointCloudOperation::AddLoadPointCloudToItems(PointCloudXYZRGB::Ptr cloud)
{
	ObjectTransformationData *item = new ObjectTransformationData();
	pcl::copyPointCloud(*cloud, *item->object_pointcloud);
	items.push_back(item);

}

void PointCloudOperation::CalculateContainerTransformation()
{
	cout << "CalculateContainerTransformation" << endl;

	container->transform->CalculateTransformation(container->object_pointcloud,0.005);
	container->transform->PrintTransformationData();

	container->CalculateWDH();
}
void PointCloudOperation::CalculateItemsTransformation()
{
	cout << "CalculateContainerTransformation" << endl;

	for (int i = 0; i < items.size(); i++)
	{
		cout << "item " << i << endl;

		items[i]->transform->CalculateTransformation(items[i]->object_pointcloud,0.0);
		items[i]->transform->PrintTransformationData();

		items[i]->CalculateWDH();
		//cout << endl;

	}
}
void PointCloudOperation::TranslatePointCloud(PointCloudXYZRGB::Ptr cloud, float translate_x, float translate_y, float translate_z)
{
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << translate_x, translate_y, translate_z;

	pcl::transformPointCloud(*cloud, *cloud, transform);
}

void PointCloudOperation::MovePointCloudFromTo(PointCloudXYZRGB::Ptr cloud, PointTypeXYZRGB current_pos, PointTypeXYZRGB target_pos)
{
	cout << "MovePointCloudFrom " << current_pos << " To " << target_pos << endl;

	double translate_x = target_pos.x - current_pos.x;
	double translate_y = target_pos.y - current_pos.y;
	double translate_z = target_pos.z - current_pos.z;

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << translate_x, translate_y, translate_z;

	pcl::transformPointCloud(*cloud, *cloud, transform);



/*	for (int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x += translate_x;
		cloud->points[i].y += translate_y;
		cloud->points[i].z += translate_z;
	}
	*/
}

void PointCloudOperation::RotatePointCloudAtAxis(PointCloudXYZRGB::Ptr cloud,
	Eigen::Matrix<float, 1, 3>  floor_plane_normal_vector,
	Eigen::Matrix<float, 1, 3>  target_plane_normal_vector)
{
	PointCloudXYZRGB::Ptr rotatedCloud(new PointCloudXYZRGB);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//transform_angle.rotate(Eigen::AngleAxisf((angle)*M_PI / 180, axis));
	
	//Calculate the rotation angle. 
	//Angle between the planes is equal to angle between the normals. 
	//From the definition of the dot product, 
	//we can extract the angle between two normal vectors. 
	//In case of XY plane, it is equal to theta=acos(C/sqrt(A^2+B^2+C^2) 
	//where A, B, C are first three coefficients of floor_plane. 

	Eigen::Matrix<float, 1, 3>  rotation_vector;

	//vector/axis to rotate around
	rotation_vector = floor_plane_normal_vector.cross(target_plane_normal_vector);
	//rotation_vector = target_plane_normal_vector.cross(floor_plane_normal_vector);
	//Error	3	error C2338: THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE	
//c:\program files\pcl172qt\eigen-3.7.2\eigen\src\Geometry\OrthoMethods.h	28	1	bpp_projectv3

	cout << "Rotation Vector(perpendicular): " << rotation_vector << std::endl;
	
		/*
		angle between 2 planes is EQUAL
		to the acute angle of normal vector of 2 planes

		equation : n1 dot n2 = length(n1)*length(n2)*cos(angle)

		*/
	float theta = acos(floor_plane_normal_vector.dot(target_plane_normal_vector) / sqrt(pow(floor_plane_normal_vector[0], 2) + pow(floor_plane_normal_vector[1], 2) + pow(floor_plane_normal_vector[2], 2)));

	transform.rotate(Eigen::AngleAxisf(theta, rotation_vector));
	
	cout << "theta=" << theta << " rad" << endl;
	cout << "theta=" << theta*180/M_PI << " deg" << endl;

	pcl::transformPointCloud(*cloud, *cloud, transform);


}

void PointCloudOperation::RotatePointCloud(PointCloudXYZRGB::Ptr cloud,
		float degree, Eigen::Matrix<float, 1, 3>  rotation_vector)
{
	cout << "Rotation Vector(perpendicular): " << rotation_vector << std::endl;
	cout << "degree=" << degree << " deg" << endl;
	cout << "degree=" << degree * M_PI / 180 << " rad" << endl;

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(degree*M_PI/180, rotation_vector));
	pcl::transformPointCloud(*cloud, *cloud, transform);

}

void PointCloudOperation::ApplyPassthroughFilter(PointCloudXYZRGB::Ptr cloud,
	double xmin, double xmax,
	double ymin, double ymax,
	double zmin, double zmax)
{
	if (cloud->size() <= 0)
	{
		QMessageBox::information(0, QString("ApplyPassthroughFilter"), QString("No PointCloud Data"), QMessageBox::Ok);
		return;

	}

	planeseg->Filter(cloud, "x", xmin, xmax);
	planeseg->Filter(cloud, "y", ymin, ymax);
	planeseg->Filter(cloud, "z", zmin, zmax);

}
void PointCloudOperation::SurfaceFillCloud(PointCloudXYZRGB::Ptr cloud, float leaf_size, float x_length, float y_length, float z_length)
{
	if (cloud->size() <= 0)
	{
		QMessageBox::information(0, QString("DuplicateInvertCloud"), QString("No PointCloud Data"), QMessageBox::Ok);
		return;
	}

	
	PointCloudXYZRGB::Ptr surface_cloud(new PointCloudXYZRGB);
	cout << "cloud->points.size() = " << cloud->points.size() << endl;

	
//	for (int i = 0; i < cloud->points.size(); i++)
	PointTypeXYZRGB minpoint, maxpoint;
	pcl::getMinMax3D(*cloud, minpoint, maxpoint);
	cout << "maxpoint " << maxpoint << endl;
	cout << "leaf_size " << leaf_size << endl;
	cout << endl;

	//method 2
	//rotate cloud to 6 face -> concatenate them
	//remove point that not in dimension area


	
	//method 1
	float border_x_max = maxpoint.x - leaf_size*10;
	float border_x_min = leaf_size*10;
	float border_z_max = maxpoint.z - leaf_size*10;
	float border_z_min = leaf_size*10;
	
	//search for point at border
	// loopy-- untily=0, add it to surface_cloud 
	//point size from 2000 -> become 30000...
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x > border_x_max || cloud->points[i].x < border_x_min || 
			cloud->points[i].z > border_z_max || cloud->points[i].z < border_z_min)
		{
			PointTypeXYZRGB newpoint = cloud->points[i];
			do
			{
				surface_cloud->push_back(newpoint);
				newpoint.y -= leaf_size;

			}while (newpoint.y >= 0.0);
		}
	}

	

	*cloud += *surface_cloud;
	cout << "cloud->points.size() = " << cloud->points.size() << endl;

}

void PointCloudOperation::DuplicateInvertCloud(PointCloudXYZRGB::Ptr cloud, float x_length, float y_length, float z_length)
{
	if (cloud->size() <= 0)
	{
		QMessageBox::information(0, QString("DuplicateInvertCloud"), QString("No PointCloud Data"), QMessageBox::Ok);
		return;
	}

	PointCloudXYZRGB::Ptr invert_cloud(new PointCloudXYZRGB);
	PointCloudXYZRGB::Ptr concate_cloud(new PointCloudXYZRGB);

	pcl::copyPointCloud(*cloud, *invert_cloud);

	RotatePointCloud(invert_cloud, 180, { 0, 0, 1 });
	TranslatePointCloud(invert_cloud, x_length, y_length, 0);

	*cloud += *invert_cloud;
	//pcl::concatenateFields(cloud, invert_cloud, concate_cloud);//a+b=c
	cout << "cloud->points.size() = " << cloud->points.size() << endl;


}