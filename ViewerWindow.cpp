#include "ViewerWindow.h"

ViewerWindow::ViewerWindow()
{
	cout << "ViewerWindow()" << endl;

	cv::namedWindow("kinect_rgb");
	cvMoveWindow("kinect_rgb", 0, 0);

	window_view.reset(new pcl::visualization::PCLVisualizer("window_view"));
	window_view->setPosition(50,0);//window position
	window_view->setBackgroundColor(0,0,0);
	window_view->initCameraParameters();
	window_view->setRepresentationToSurfaceForAllActors();

}

void ViewerWindow::SetDataProcess(DataProcess* d) {dataprocess = d;}

void ViewerWindow::SetCameraParameter(
	double focal_x, double focal_y, double focal_z,
	double pos_x, double pos_y, double pos_z,
	double up_x, double up_y, double up_z,
	double clipping_near, double clipping_far,
	double cam_angle_rad, double cam_angle_deg)
{
	window_view->setCameraPosition(
		pos_x, pos_y, pos_z,
		focal_x, focal_y, focal_z,
		up_x, up_y, up_z);

	window_view->setCameraClipDistances(clipping_near, clipping_far);
	window_view->setCameraFieldOfView(cam_angle_rad);//radian
}

void ViewerWindow::UpdateWindowCloudViewer(PointCloudXYZRGB::Ptr pointcloud)
{
	if (!window_view->updatePointCloud(pointcloud, "window_view"))
	{
		window_view->addPointCloud(pointcloud, "window_view");
	}
	window_view->spinOnce();
}
void ViewerWindow::UpdateWindowRGB(cv::Mat image)
{
	cv::imshow("kinect_rgb", image);
}

void ViewerWindow::ClearPointCloudWindowCloudViewer()
{
	window_view->removeAllPointClouds();
	window_view->spinOnce();
}
void ViewerWindow::ClearShapeWindowCloudViewer()
{
	window_view->removeAllShapes();
	window_view->spinOnce();
}

void ViewerWindow::AddPolygonMesh(
	float w, float h, float d,
	float x, float y, float z,
	float r, float g, float b)
{
	float xmin = x;
	float ymin = y;
	float zmin = z;
	float xmax = (x + w);
	float ymax = (y + h);
	float zmax = (z + d);

	pcl::PointXYZ p1, p2, p3, p4, p5, p6, p7, p8;

	p1.x = xmin; p1.y = ymin; p1.z = zmin;
	p2.x = xmax; p2.y = ymin; p2.z = zmin;
	p3.x = xmin; p3.y = ymin; p3.z = zmax;
	p4.x = xmax; p4.y = ymin; p4.z = zmax;

	p5.x = xmin; p5.y = ymax; p5.z = zmin;
	p6.x = xmax; p6.y = ymax; p6.z = zmin;
	p7.x = xmin; p7.y = ymax; p7.z = zmax;
	p8.x = xmax; p8.y = ymax; p8.z = zmax;

	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;

	//pcl::copyPointCloud( *pointcloud, *cloud_blob);
	//pcl::io::loadPCDFile("C:/Users/Nattaon/Desktop/bpp_project_december/bun0.pcd", *cloud);
	cloud->points.push_back(p1);
	cloud->points.push_back(p2);
	cloud->points.push_back(p3);
	cloud->points.push_back(p4);
	cloud->points.push_back(p5);
	cloud->points.push_back(p6);
	cloud->points.push_back(p7);
	cloud->points.push_back(p8);
	cloud->width = (int)cloud->points.size();
	cloud->height = 1;
	
	window_view->addPointCloud(cloud, "window_view");
	//pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	/*triangles.polygons.resize(2);
	triangles.polygons[0].vertices.push_back(0);
	triangles.polygons[0].vertices.push_back(1);
	triangles.polygons[0].vertices.push_back(2);
	triangles.polygons[1].vertices.push_back(1);
	triangles.polygons[1].vertices.push_back(2);
	triangles.polygons[1].vertices.push_back(3);

	pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
	mesh_cld.push_back(p1);
	mesh_cld.push_back(p2);
	mesh_cld.push_back(p3);
	mesh_cld.push_back(p4);

	pcl:toROSMsg(mesh_cld, triangles.cloud);*/

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(w*2);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	// bottom face
	triangles.polygons[0].vertices[0] = 0;
	triangles.polygons[0].vertices[1] = 1;
	triangles.polygons[0].vertices[2] = 2;

	triangles.polygons[1].vertices[0] = 3;
	triangles.polygons[1].vertices[1] = 1;
	triangles.polygons[1].vertices[2] = 2;
	// left face
	triangles.polygons[2].vertices[0] = 1;
	triangles.polygons[2].vertices[1] = 3;
	triangles.polygons[2].vertices[2] = 5;

	triangles.polygons[3].vertices[0] = 7;
	triangles.polygons[3].vertices[1] = 3;
	triangles.polygons[3].vertices[2] = 5;
	// rigth face
	triangles.polygons[4].vertices[0] = 0;
	triangles.polygons[4].vertices[1] = 4;
	triangles.polygons[4].vertices[2] = 2;

	triangles.polygons[5].vertices[0] = 6;
	triangles.polygons[5].vertices[1] = 4;
	triangles.polygons[5].vertices[2] = 2;
	// front face
	triangles.polygons[6].vertices[0] = 0;
	triangles.polygons[6].vertices[1] = 1;
	triangles.polygons[6].vertices[2] = 4;

	triangles.polygons[7].vertices[0] = 5;
	triangles.polygons[7].vertices[1] = 1;
	triangles.polygons[7].vertices[2] = 4;
	//back face
	triangles.polygons[8].vertices[0] = 2;
	triangles.polygons[8].vertices[1] = 3;
	triangles.polygons[8].vertices[2] = 6;

	triangles.polygons[9].vertices[0] = 7;
	triangles.polygons[9].vertices[1] = 3;
	triangles.polygons[9].vertices[2] = 6;
	// top face
	triangles.polygons[10].vertices[0] = 4;
	triangles.polygons[10].vertices[1] = 5;
	triangles.polygons[10].vertices[2] = 6;

	triangles.polygons[11].vertices[0] = 7;
	triangles.polygons[11].vertices[1] = 5;
	triangles.polygons[11].vertices[2] = 6;
	window_view->addPolygonMesh(triangles, "triangles");
	window_view->spinOnce();
}



void ViewerWindow::DrawPlanarAtOrigin(double plane_halflegth,
	double r, double g, double b, string planename)
{
	double minus_halflength = -1 * plane_halflegth;
	PointTypeXYZRGB p1, p2, p3, p4;

	p1.x = minus_halflength; p1.y = 0; p1.z = minus_halflength;
	p2.x = minus_halflength; p2.y = 0; p2.z = plane_halflegth;
	p3.x = plane_halflegth; p3.y = 0; p3.z = plane_halflegth;
	p4.x = plane_halflegth; p4.y = 0; p4.z = minus_halflength;

	PointCloudXYZRGB::Ptr polygon_pointcloud;//= new PointCloudXYZRGB();
	polygon_pointcloud.reset(new PointCloudXYZRGB);

	polygon_pointcloud->points.push_back(p1);
	polygon_pointcloud->points.push_back(p2);
	polygon_pointcloud->points.push_back(p3);
	polygon_pointcloud->points.push_back(p4);

	polygon_pointcloud->width = (int)polygon_pointcloud->points.size();
	polygon_pointcloud->height = 1;
	
	//cout << "polygon_pointcloud" << polygon_pointcloud->points.size() << endl;

	window_view->removeShape(planename);
	window_view->addPolygon<PointTypeXYZRGB>(polygon_pointcloud, r,g,b, planename);
	window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, planename);
	window_view->spinOnce();

}

void ViewerWindow::AddBoundingBoxWindowCloudViewer(PointTypeXYZRGB position_OBB, 
	PointTypeXYZRGB min_point_OBB, PointTypeXYZRGB max_point_OBB, 
	Eigen::Matrix3f rotational_matrix_OBB, string cloudname)
{
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	window_view->removeShape(cloudname);
	window_view->addCube(position, quat, 
		max_point_OBB.x - min_point_OBB.x, 
		max_point_OBB.y - min_point_OBB.y, 
		max_point_OBB.z - min_point_OBB.z, cloudname);

}

void ViewerWindow::AddVectorDirectionWindowCloudViewer(Eigen::Vector3f mass_center, 
	Eigen::Vector3f major_vector, Eigen::Vector3f middle_vector, Eigen::Vector3f minor_vector, 
	string cloudname)
{
	//cout << "AddVectorDirectionWindowCloudViewer" << endl;
	float vector_scale = 0.1f;

	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(
		major_vector(0)*vector_scale + mass_center(0), 
		major_vector(1)*vector_scale + mass_center(1), 
		major_vector(2)*vector_scale + mass_center(2));
	pcl::PointXYZ y_axis(
		middle_vector(0)*vector_scale + mass_center(0), 
		middle_vector(1)*vector_scale + mass_center(1), 
		middle_vector(2)*vector_scale + mass_center(2));
	pcl::PointXYZ z_axis(
		minor_vector(0)*vector_scale + mass_center(0), 
		minor_vector(1)*vector_scale + mass_center(1), 
		minor_vector(2)*vector_scale + mass_center(2));

	window_view->removeShape(cloudname + " major eigen vector");
	window_view->removeShape(cloudname + " middle eigen vector");
	window_view->removeShape(cloudname + " minor eigen vector");

	window_view->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, cloudname + " major eigen vector");
	window_view->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, cloudname + " middle eigen vector");
	window_view->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, cloudname + " minor eigen vector");
}
void ViewerWindow::AddTextWindowCloudViewer(PointTypeXYZRGB point_position, double text_scale,
	double r, double g, double b, string drawtext, string cloudname)
{
	window_view->removeText3D(cloudname);
	window_view->addText3D(drawtext, point_position, text_scale, r, g, b, cloudname);
}

void ViewerWindow::AddSphereWindowCloudViewer(PointTypeXYZRGB point_position, double radius, double r, double g, double b, string id_name)
{
	window_view->removeShape(id_name);
	window_view->addSphere(point_position, radius, r, g, b, id_name);
}



void ViewerWindow::AddSymbolWindowCloudViewer(
	PointTypeXYZRGB position_OBB, 
	PointTypeXYZRGB min3d, 
	PointTypeXYZRGB max3d,
	Eigen::Vector3f mass_center,
	Eigen::Vector3f major_vector,
	Eigen::Vector3f middle_vector,
	double r, double g, double b, string cloudname)
{
	PointTypeXYZRGB position_major;
	PointTypeXYZRGB position_minor_plus, position_minor_minus;
	PointTypeXYZRGB position_center;
	PointCloudXYZRGB::Ptr polygon_pointcloud;//= new PointCloudXYZRGB();
	polygon_pointcloud.reset(new PointCloudXYZRGB);

	float vector_scale = 0.1f;
	float half_length_x = (max3d.x - min3d.x)*0.5;
	float half_length_y = (max3d.y - min3d.y)*0.5;
	float half_length_z = (max3d.z - min3d.z)*0.5;//most small=(height)
	float translate_pos = 0.01;// for draw symbol on top of item

	/*cout << endl;
	cout << cloudname << endl;
	cout << "length_x= " << half_length_x << ", major(0)= " << major_vector(0) << ", middle(0)= " << middle_vector(0) << endl;
	cout << "length_y= " << half_length_y << ", major(1)= " << major_vector(1) << ", middle(1)= " << middle_vector(1) << endl;
	cout << "length_z= " << half_length_z << ", major(2)= " << major_vector(2) << ", middle(2)= " << middle_vector(2) << endl;
	*/
	//cout << "translate_pos= " << translate_pos << endl;

	position_center.x = position_OBB.x;
	position_center.y = position_OBB.y + translate_pos;
	position_center.z = position_OBB.z;

	position_major.x = (position_OBB.x + major_vector(0)*half_length_x);
	position_major.y = (position_OBB.y + major_vector(1)*half_length_y) + translate_pos;
	position_major.z = (position_OBB.z + major_vector(2)*half_length_z);

	//window_view->addLine(position_center, position_major, 0.0f, 0.0f, 1.0f, cloudname + " majorline");
	//window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, cloudname + " majorline");

	position_minor_plus.x = (position_OBB.x + middle_vector(0)*half_length_x);
	position_minor_plus.y = (position_OBB.y + middle_vector(1)*half_length_y) + translate_pos;
	position_minor_plus.z = (position_OBB.z + middle_vector(2)*half_length_z);
	position_minor_minus.x = (position_OBB.x - middle_vector(0)*half_length_x);
	position_minor_minus.y = (position_OBB.y - middle_vector(1)*half_length_y) + translate_pos;
	position_minor_minus.z = (position_OBB.z - middle_vector(2)*half_length_z);


	/*cout << "position_center " << position_center << endl;
	cout << "position_major " << position_major << endl;
	cout << "position_minor_plus " << position_minor_plus << endl;
	*/
	//window_view->addLine(position_center, position_minor_plus, 1.0f, 0.0f, 1.0f, cloudname + " minorlineplus");
	

	polygon_pointcloud->points.push_back(position_major);
	polygon_pointcloud->points.push_back(position_minor_minus);
	polygon_pointcloud->points.push_back(position_minor_plus);
	
	polygon_pointcloud->width = (int)polygon_pointcloud->points.size();
	polygon_pointcloud->height = 1;

	window_view->removeShape(cloudname + " polygon");
	window_view->addPolygon<PointTypeXYZRGB>(polygon_pointcloud, 1.0f, 0.0f, 0.0f, cloudname + " polygon");
	window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cloudname + " polygon");

}

void ViewerWindow::AddCircleWindowCloudViewer(
	PointTypeXYZRGB position_OBB, float radius,
	double r, double g, double b, string cloudname)
{
	/* //circle border only not fill inside
	pcl::ModelCoefficients circle_coeff;
	circle_coeff.values.resize(3);    // We need 3 values
	circle_coeff.values[0] = position_OBB.x;
	circle_coeff.values[1] = position_OBB.z;
	circle_coeff.values[2] = radius;
	window_view->addCircle(circle_coeff, cloudname);*/

	PointCloudXYZRGB::Ptr polygon_pointcloud;//= new PointCloudXYZRGB();
	polygon_pointcloud.reset(new PointCloudXYZRGB);
	
	//cout << "radius=" << radius << endl;
	int circle_segment = 12;
	for (int i = 0; i < circle_segment; i++)
	{
		float theta = 2.0 * M_PI*float(i) / float(circle_segment);
		float x = radius * cosf(theta);//calculate the x component
		float y = radius * sinf(theta);//calculate the y component

		//cout << "boundery x=" << x << endl;
		//cout << "boundery y=" << y << endl;

		PointTypeXYZRGB boundery_point;
		boundery_point.x = position_OBB.x+x;
		boundery_point.y = position_OBB.y;
		boundery_point.z = position_OBB.z+y;

		//cout << "boundery_point = " << boundery_point << endl;

		polygon_pointcloud->points.push_back(boundery_point);
	}
	polygon_pointcloud->width = (int)polygon_pointcloud->points.size();
	polygon_pointcloud->height = 1;

	window_view->removeShape(cloudname + " polygon");
	window_view->addPolygon<PointTypeXYZRGB>(polygon_pointcloud, r, g, b, cloudname + " polygon");
	window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cloudname + " polygon");


}








void ViewerWindow::ToggleAxisONWindowCloudViewer()
{
	window_view->addCoordinateSystem();
	window_view->spinOnce();
}
void ViewerWindow::ToggleAxisOFFWindowCloudViewer()
{
	window_view->removeCoordinateSystem();
	window_view->spinOnce();
}













inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v, Eigen::Vector3f rgb) { pcl::PointXYZRGB p(rgb[0], rgb[1], rgb[2]); p.x = v[0]; p.y = v[1]; p.z = v[2]; return p; }

pcl::PolygonMesh ViewerWindow::visualizerGetCameraMesh(const Eigen::Matrix3f& R, const Eigen::Vector3f& t, 
	float r, float g, float b,
	Eigen::Vector3f& vforward, Eigen::Vector3f& rgb)
{
	
	double s = 0.01;
	int	ipolygon[18] = { 0, 1, 2, 0, 3, 1, 0, 4, 3, 0, 2, 4, 3, 1, 4, 2, 4, 1 };

	Eigen::Vector3f vright = R.row(0).normalized() * s;
	Eigen::Vector3f vup = -R.row(1).normalized() * s;
	vforward = R.row(2).normalized() * s;

	rgb = Eigen::Vector3f(r, g, b);

	pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
	mesh_cld.push_back(Eigen2PointXYZRGB(t, rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright / 2.0 + vup / 2.0, rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright / 2.0 - vup / 2.0, rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright / 2.0 + vup / 2.0, rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright / 2.0 - vup / 2.0, rgb));

	pcl::PolygonMesh pm;
	pm.polygons.resize(6);
	for (int i = 0; i<6; i++)
		for (int _v = 0; _v<3; _v++)
			pm.polygons[i].vertices.push_back(ipolygon[i * 3 + _v]);
	pcl::toROSMsg(mesh_cld, pm.cloud);

	return pm;
}

void ViewerWindow::DrawItemCubeShader(float w, float h, float d,
	float x, float y, float z,
	int r, int g, int b,
	string shapename)
{
	float xmin = x;
	float ymin = y;
	float zmin = z;
	float xmax = (x + w);
	float ymax = (y + h);
	float zmax = (z + d);

	pcl::PointXYZRGB p1(r, g, b), p2(r, g, b), p3(r, g, b), p4(r, g, b), p5(r, g, b), p6(r, g, b), p7(r, g, b), p8(r, g, b);

	p1.x = xmin; p1.y = ymin; p1.z = zmin;
	p2.x = xmax; p2.y = ymin; p2.z = zmin;
	p3.x = xmin; p3.y = ymin; p3.z = zmax;
	p4.x = xmax; p4.y = ymin; p4.z = zmax;

	p5.x = xmin; p5.y = ymax; p5.z = zmin;
	p6.x = xmax; p6.y = ymax; p6.z = zmin;
	p7.x = xmin; p7.y = ymax; p7.z = zmax;
	p8.x = xmax; p8.y = ymax; p8.z = zmax;

	pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->points.push_back(p1);
	cloud->points.push_back(p2);
	cloud->points.push_back(p3);
	cloud->points.push_back(p4);
	cloud->points.push_back(p5);
	cloud->points.push_back(p6);
	cloud->points.push_back(p7);
	cloud->points.push_back(p8);
	cloud->width = (int)cloud->points.size();
	cloud->height = 1;

	int	ipolygon[36] = {0, 1, 2,// bottom face
						3, 1, 2, 
						1, 3, 5,// left face
						7, 3, 5, 
						0, 4, 2,// rigth face
						6, 4, 2,
						0, 1, 4,// front face
						5, 1, 4,
						2, 3, 6,// back face
						7, 3, 6,
						4, 5, 6,// top face
						7, 5, 6};
	int face_num = 12;
	pcl::PolygonMesh triangles;
	triangles.polygons.resize(face_num);
	for (int i = 0; i<face_num; i++)
		for (int _v = 0; _v<3; _v++)
			triangles.polygons[i].vertices.push_back(ipolygon[i * 3 + _v]);



	pcl::toROSMsg(*cloud, triangles.cloud);

	window_view->addPolygonMesh(triangles, shapename);
	//window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, shapename); //cannotset on polygonmesh
	window_view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, shapename);
	window_view->spinOnce();
}

void ViewerWindow::DrawItemCube(float w, float h, float d, 
	float x, float y, float z, 
	float r, float g, float b,
	string shapename)
{

	
	float xmin = x;
	float ymin = y;
	float zmin = z;
	float xmax = (x + w);
	float ymax = (y + h);
	float zmax = (z + d);

	cout << endl;
	cout << r << "," << g << "," << b << endl; 
	cout << "x " << xmin << "," << xmax << endl;
	cout << "y " << ymin << "," << ymax << endl;
	cout << "z " << zmin << "," << zmax << endl;

	window_view->removeShape(shapename);
	window_view->addCube(xmin,xmax,ymin,ymax,zmin,zmax,r,g,b,shapename);
	window_view->spinOnce();
	/*
	Eigen::Matrix3f rotational_matrix_OBB;
	rotational_matrix_OBB(0) = 1.0f;
	rotational_matrix_OBB(1) = 0.0f;
	rotational_matrix_OBB(2) = 0.0f;

	rotational_matrix_OBB(3) = 0.0f;
	rotational_matrix_OBB(4) = 1.0f;
	rotational_matrix_OBB(5) = 0.0f;

	rotational_matrix_OBB(6) = 0.0f;
	rotational_matrix_OBB(7) = 0.0f;
	rotational_matrix_OBB(8) = 1.0f;



	Eigen::Vector3f position(xmin, ymin, zmin);
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	
	window_view->addCube(position, quat, w, h, d, shapename + " polygon");
	
*/
	
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints = tree_ptr->crown->hull_cloud;
	std::vector<pcl::Vertices> polygons = tree_ptr->crown->polygons;
	viewer->addPolygonMesh<pcl::PointXYZ>(hullPoints, polygons, "crown");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0, "crown");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "crown");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "crown");
*/
}
void ViewerWindow::DrawItemArrowDirectionSymbol(
	float w, float h, float d,
	float x, float y, float z,
	float r, float g, float b,
	string symbolname)
{
	PointTypeXYZRGB position_major;
	PointTypeXYZRGB position_minor_plus, position_minor_minus;
	PointTypeXYZRGB position_center;
	PointCloudXYZRGB::Ptr polygon_pointcloud;//= new PointCloudXYZRGB();
	polygon_pointcloud.reset(new PointCloudXYZRGB);

	float half_length_x = w*0.5;
	float half_length_y = h*0.5;
	float half_length_z = d*0.5;//most small=(height)
	float translate_pos = half_length_y + 0.01;// for draw symbol on top of item

	/*cout << endl;
	cout << cloudname << endl;
	cout << "length_x= " << half_length_x << ", major(0)= " << major_vector(0) << ", middle(0)= " << middle_vector(0) << endl;
	cout << "length_y= " << half_length_y << ", major(1)= " << major_vector(1) << ", middle(1)= " << middle_vector(1) << endl;
	cout << "length_z= " << half_length_z << ", major(2)= " << major_vector(2) << ", middle(2)= " << middle_vector(2) << endl;
	*/
	//cout << "translate_pos= " << translate_pos << endl;

	position_center.x = x + half_length_x;
	position_center.y = y + half_length_y;
	position_center.z = z + half_length_z;

	//y is height direction : no effect
	//decide which x or z has a longer side
	if (half_length_x > half_length_z)
	{
		position_major.x = position_center.x + half_length_x;
		position_major.y = position_center.y + translate_pos;
		position_major.z = position_center.z;

		position_minor_plus.x = position_center.x;
		position_minor_plus.y = position_center.y + translate_pos;
		position_minor_plus.z = position_center.z + half_length_z;

		position_minor_minus.x = position_center.x;
		position_minor_minus.y = position_center.y + translate_pos;
		position_minor_minus.z = position_center.z - half_length_z;


		polygon_pointcloud->points.push_back(position_major);
		polygon_pointcloud->points.push_back(position_minor_minus);
		polygon_pointcloud->points.push_back(position_minor_plus);
	}
	else
	{

		position_major.x = position_center.x;
		position_major.y = position_center.y + translate_pos;
		position_major.z = position_center.z + half_length_z;

		position_minor_plus.x = position_center.x + half_length_x;
		position_minor_plus.y = position_center.y + translate_pos;
		position_minor_plus.z = position_center.z;

		position_minor_minus.x = position_center.x - half_length_x;
		position_minor_minus.y = position_center.y + translate_pos;
		position_minor_minus.z = position_center.z;


		polygon_pointcloud->points.push_back(position_major);
		polygon_pointcloud->points.push_back(position_minor_minus);
		polygon_pointcloud->points.push_back(position_minor_plus);
	}



	polygon_pointcloud->width = (int)polygon_pointcloud->points.size();
	polygon_pointcloud->height = 1;

	window_view->removeShape(symbolname + " polygon");
	window_view->addPolygon<PointTypeXYZRGB>(polygon_pointcloud, 1.0f, 0.0f, 0.0f, symbolname + " polygon");
	window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, symbolname + " polygon");

}

void ViewerWindow::randomcolorfloat(float &r, float &g, float &b)
{
	int r256 = rand() % 200 + 56;
	int g256 = rand() % 200 + 56;
	int b256 = rand() % 200 + 56;


	r = r256 / 255.0;
	g = g256 / 255.0;
	b = b256 / 255.0;

	//int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);


	//cout << "color = " << rgb << endl; //6388670

	
}
void ViewerWindow::randomcolorint(int &r, int &g, int &b)
{
	int r256 = rand() % 200 + 56;
	int g256 = rand() % 200 + 56;
	int b256 = rand() % 200 + 56;


	r = r256;
	g = g256;
	b = b256;

	//int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);


	//cout << "color = " << rgb << endl; //6388670


}


void ViewerWindow::ShowBinPackingTarget(ObjectTransformationData *container, vector<ObjectTransformationData*> items)
{
	//cout << "container_position=" << container_position << endl;

	Eigen::Matrix<float, 1, 3>  rotation_x_axis(1.0, 0.0, 0.0);
	Eigen::Matrix<float, 1, 3>  rotation_y_axis(0.0, 1.0, 0.0);
	Eigen::Matrix<float, 1, 3>  rotation_z_axis(0.0, 0.0, 1.0);

	for (int i = 0; i < items.size(); i++)
	{
		string cloudname = "itemcloud" + std::to_string(i);
	

		cout << "cloudname " << cloudname << endl;


		PointCloudXYZRGB::Ptr item_pointcloud;
		item_pointcloud.reset(new PointCloudXYZRGB);
		pcl::copyPointCloud(*items[i]->object_pointcloud, *item_pointcloud);

		//if (items[i]->rotation_case == 0) // do nothing
		if (items[i]->rotation_case == 1)
		{
			dataprocess->RotatePointCloud(item_pointcloud,
							-90, rotation_x_axis);
			dataprocess->TranslatePointCloud(item_pointcloud,
				0.0, 0.0, items[i]->input_dimension.y);

		}
		else if (items[i]->rotation_case == 2)
		{
			dataprocess->RotatePointCloud(item_pointcloud,
				90, rotation_z_axis);
			dataprocess->TranslatePointCloud(item_pointcloud,
				items[i]->input_dimension.y, 0.0, 0.0);
		}
		else if (items[i]->rotation_case == 3)
		{
			dataprocess->RotatePointCloud(item_pointcloud,
				90, rotation_x_axis);
			dataprocess->RotatePointCloud(item_pointcloud,
				90, rotation_z_axis);
		}
		else if (items[i]->rotation_case == 4)
		{
			dataprocess->RotatePointCloud(item_pointcloud,
				90, rotation_y_axis);
			dataprocess->TranslatePointCloud(item_pointcloud,
				0.0, 0.0, items[i]->input_dimension.x);
		}
		else if (items[i]->rotation_case == 5)
		{
			dataprocess->RotatePointCloud(item_pointcloud,
				-90, rotation_y_axis);
			dataprocess->RotatePointCloud(item_pointcloud,
				-90, rotation_z_axis);
		}

		dataprocess->TranslatePointCloud(item_pointcloud,
			items[i]->target_position.x, items[i]->target_position.y, items[i]->target_position.z);

		dataprocess->TranslatePointCloud(item_pointcloud,
			container->transform->min3d_point.x, container->transform->min3d_point.y, container->transform->min3d_point.z);


		if (!window_view->updatePointCloud(item_pointcloud, cloudname))
		{
			window_view->addPointCloud(item_pointcloud, cloudname);
		}

		window_view->spinOnce();

	
	}
	
}



void ViewerWindow::ShowBinpackingIndication(ObjectTransformationData *container, vector<ObjectTransformationData*> items)
{
	//cout << "ref_position= " << ref_position << endl;
	/*PointTypeXYZRGB point_sphere;
	point_sphere.x = 0;
	point_sphere.y = 0;
	point_sphere.z = 0;
	AddSphereWindowCloudViewer(point_sphere,0.3,1.0,0,0,"point_sphere");*/

	for (int i = 0; i < items.size(); i++)
	{
		string input_cube_name = "input_cube_name" + std::to_string(i);
		string input_symbol_name = "input_symbol_name" + std::to_string(i);
		string target_cube_name = "target_cube_name" + std::to_string(i);
		string target_symbol_name = "target_symbol_name" + std::to_string(i);
		string line_name = "line_name" + std::to_string(i);

		float in_cube_x_dim = items[i]->x_length;
		float in_cube_y_dim = items[i]->y_length;
		float in_cube_z_dim = items[i]->z_length;
		float in_cube_x_min_pos = items[i]->transform->min3d_point.x;
		float in_cube_y_min_pos = items[i]->transform->min3d_point.y;
		float in_cube_z_min_pos = items[i]->transform->min3d_point.z;

		float tar_cube_x_dim = items[i]->target_orientation.x * 0.001;
		float tar_cube_y_dim = items[i]->target_orientation.y * 0.001;
		float tar_cube_z_dim = items[i]->target_orientation.z * 0.001;
		float tar_cube_x_min_pos = items[i]->target_position.x + container->transform->min3d_point.x;
		float tar_cube_y_min_pos = items[i]->target_position.y + container->transform->min3d_point.y;
		float tar_cube_z_min_pos = items[i]->target_position.z + container->transform->min3d_point.z;
		
		PointTypeXYZRGB input_center_symbol;
		input_center_symbol.x = in_cube_x_min_pos + in_cube_x_dim*0.5;
		input_center_symbol.y = in_cube_y_min_pos + in_cube_y_dim;
		input_center_symbol.z = in_cube_z_min_pos + in_cube_z_dim*0.5;

		PointTypeXYZRGB target_center_symbol;
		target_center_symbol.x = tar_cube_x_min_pos + tar_cube_x_dim*0.5;
		target_center_symbol.y = tar_cube_y_min_pos + tar_cube_y_dim;
		target_center_symbol.z = tar_cube_z_min_pos + tar_cube_z_dim*0.5;

		/*cout << "input_symbol_name " << input_symbol_name << endl;
		cout << "pos x,y,z= " << cube_x_min_pos << "," << cube_y_min_pos << "," << cube_z_min_pos << endl;
		cout << "dim x,y,z = " << cube_x_dim << "," << cube_y_dim << "," << cube_z_dim << endl;
		cout << endl;*/

		int r, g, b;
		randomcolorint(r, g, b);

		//input cube
		DrawItemCubeShader(
			in_cube_x_dim, in_cube_y_dim, in_cube_z_dim,
			in_cube_x_min_pos, in_cube_y_min_pos, in_cube_z_min_pos,
			r, g, b,
			input_cube_name);

/*		window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, input_cube_name);

		window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,
			pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, input_cube_name);
*/

		//input direction symbol
		DrawItemArrowDirectionSymbol(
			in_cube_x_dim, in_cube_y_dim, in_cube_z_dim,
			in_cube_x_min_pos, in_cube_y_min_pos, in_cube_z_min_pos,
			1.0, 1.0, 1.0, input_symbol_name);

		//target cube
		DrawItemCubeShader(
			tar_cube_x_dim, tar_cube_y_dim, tar_cube_z_dim,
			tar_cube_x_min_pos, tar_cube_y_min_pos, tar_cube_z_min_pos,
			r, g, b,
			target_cube_name);

/*		window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, target_cube_name);

		window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,
			pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, target_cube_name);
*/


		//target direction symbol
		DrawItemArrowDirectionSymbol(
			tar_cube_x_dim, tar_cube_y_dim, tar_cube_z_dim,
			tar_cube_x_min_pos, tar_cube_y_min_pos, tar_cube_z_min_pos,
			1.0, 1.0, 1.0, target_symbol_name);

		//link line between input and target
		window_view->addLine(input_center_symbol, target_center_symbol, 1.0f, 0.0f, 0.0f, line_name);
		window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, line_name);

		
		window_view->spinOnce();
	}
	
}
