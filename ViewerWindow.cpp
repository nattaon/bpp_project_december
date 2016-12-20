#include "ViewerWindow.h"

ViewerWindow::ViewerWindow()
{
	cout << "ViewerWindow()" << endl;

	cv::namedWindow("kinect_rgb");
	cvMoveWindow("kinect_rgb", 0, 0);

	window_view.reset(new pcl::visualization::PCLVisualizer("window_view"));
	window_view->setPosition(50,0);//window position
	window_view->setBackgroundColor(0, 0, 0);
	window_view->initCameraParameters();


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
}
void ViewerWindow::ClearShapeWindowCloudViewer()
{
	window_view->removeAllShapes();
}

void ViewerWindow::AddBoundingBoxWindowCloudViewer(PointTypeXYZRGB position_OBB, 
	PointTypeXYZRGB min_point_OBB, PointTypeXYZRGB max_point_OBB, 
	Eigen::Matrix3f rotational_matrix_OBB, string cloudname)
{
	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat(rotational_matrix_OBB);
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
	window_view->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, cloudname + " major eigen vector");
	window_view->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, cloudname + " middle eigen vector");
	window_view->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, cloudname + " minor eigen vector");
}
void ViewerWindow::AddTextWindowCloudViewer(PointTypeXYZRGB point_position, double text_scale,
	double r, double g, double b, string drawtext, string cloudname)
{
	window_view->addText3D(drawtext, point_position, text_scale, r, g, b, cloudname);
}

void ViewerWindow::AddSphereWindowCloudViewer(PointTypeXYZRGB point_position, double radius, double r, double g, double b, string id_name)
{
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
	window_view->addPolygon<PointTypeXYZRGB>(polygon_pointcloud, 1.0f, 0.0f, 0.0f, cloudname + " polygon");
	window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cloudname + " polygon");


	//window_view->addLine(position_center, position_minor_minus, 0.0f, 0.0f, 1.0f, cloudname + " minorlineminus");
	//window_view->addLine(position_minor_minus, position_minor_plus, 1.0f, 0.0f, 0.0f, cloudname + " minorline");
	//window_view->addLine(position_minor_minus, position_major, 0.0f, 0.0f, 1.0f, cloudname + " minorminusline");
	//window_view->addLine(position_minor_plus, position_major, 0.0f, 0.0f, 1.0f, cloudname + " minorplusline");
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


void ViewerWindow::ShowBinpacking(
	PointTypeXYZRGB ref_position, 
	int n,
	int binW, int binH, int binD, 
	int *w, int *h, int *d, 
	int *x, int *y, int *z)
{
	cout << "ref_position= " << ref_position << endl;

	for (int i = 0; i < n; i++)
	{
		string shapename = "itmecube" + std::to_string(i);
		string symbolname = "symbolitmecube" + std::to_string(i);
		cout << endl;
		cout << "shapename " << shapename << endl;
		cout << "x,y,z = " << x[i] << "," << y[i] << "," << z[i] << endl;
		cout << "w,h,d = " << w[i] << "," << h[i] << "," << d[i] << endl;
		
		float cube_w = w[i] * 0.001;
		float cube_h = h[i] * 0.001;
		float cube_d = d[i] * 0.001;
		float cube_x_min = x[i] * 0.001 + ref_position.x;
		float cube_y_min = y[i] * 0.001 + ref_position.y;
		float cube_z_min = z[i] * 0.001 + ref_position.z;


		DrawItemCube(
			cube_w, cube_h, cube_d,
			cube_x_min, cube_y_min, cube_z_min,
			shapename);


		DrawItemSymbol(
			cube_w, cube_h, cube_d,
			cube_x_min, cube_y_min, cube_z_min,
			1.0, 1.0, 1.0, symbolname);
	}
	window_view->spinOnce();
}

void ViewerWindow::DrawItemCube(float w, float h, float d, float x, float y, float z, string shapename)
{
	double r, g, b;
	randomcolor(r, g, b);
	
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

	window_view->addCube(xmin,xmax,ymin,ymax,zmin,zmax,r,g,b,shapename);
	
}
void ViewerWindow::DrawItemSymbol(
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
	window_view->addPolygon<PointTypeXYZRGB>(polygon_pointcloud, 1.0f, 0.0f, 0.0f, symbolname + " polygon");
	window_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, symbolname + " polygon");

}

void ViewerWindow::randomcolor(double &r, double &g, double &b)
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


