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
	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
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
	PointTypeXYZRGB min_point_OBB, 
	PointTypeXYZRGB max_point_OBB,
	Eigen::Vector3f mass_center,
	Eigen::Vector3f major_vector,
	Eigen::Vector3f middle_vector,
	double r, double g, double b, string cloudname)
{
	PointTypeXYZRGB position_major;
	PointTypeXYZRGB position_minor_plus, position_minor_minus;
	PointTypeXYZRGB position_center;
	

	double length_x = (max_point_OBB.x - min_point_OBB.x)*0.5;
	double length_y = (max_point_OBB.y - min_point_OBB.y)*0.5;
	double length_z = (max_point_OBB.z - min_point_OBB.z);
	double translate_pos = length_y*0.5;

	cout << "translate_pos= " << translate_pos << endl;

	position_major.x = (position_OBB.x + major_vector(0)*length_x);
	position_major.y = (position_OBB.y + major_vector(1)*length_y) + translate_pos;
	position_major.z = (position_OBB.z + major_vector(2)*length_z);

	position_minor_plus.x = (position_OBB.x + middle_vector(0)*length_x);
	position_minor_plus.y = (position_OBB.y + middle_vector(1)*length_y) + translate_pos;
	position_minor_plus.z = (position_OBB.z + middle_vector(2)*length_z);

	position_minor_minus.x = (position_OBB.x - middle_vector(0)*length_x);
	position_minor_minus.y = (position_OBB.y - middle_vector(1)*length_y) + translate_pos;
	position_minor_minus.z = (position_OBB.z - middle_vector(2)*length_z);

	position_center.x = position_OBB.x;
	position_center.y = position_OBB.y + translate_pos;
	position_center.z = position_OBB.z;


	window_view->addLine(position_center, position_minor_minus, 0.0f, 0.0f, 1.0f, cloudname + " minorlineminus");
	window_view->addLine(position_center, position_minor_plus, 0.0f, 0.0f, 1.0f, cloudname + " minorlineplus");

	window_view->addLine(position_center, position_major, 0.0f, 0.0f, 1.0f, cloudname + " majorline");
	window_view->addLine(position_minor_minus, position_minor_plus, 0.0f, 0.0f, 1.0f, cloudname + " minorline");
	window_view->addLine(position_minor_minus, position_major, 0.0f, 0.0f, 1.0f, cloudname + " minorminusline");
	window_view->addLine(position_minor_plus, position_major, 0.0f, 0.0f, 1.0f, cloudname + " minorplusline");
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


void ViewerWindow::ShowBinpacking(int n,
	int binW, int binH, int binD, 
	int *w, int *h, int *d, 
	int *x, int *y, int *z)
{
	for (int i = 0; i < n; i++)
	{
		string shapename = "itmecube" + std::to_string(i);
		DrawItemCube(w[i], h[i], d[i], x[i], y[i], z[i], shapename);
	}
	window_view->spinOnce();
}

void ViewerWindow::DrawItemCube(int w, int h, int d, int x, int y, int z, string shapename)
{
	double r, g, b;
	randomcolor(r, g, b);
	
	double xmin = x*0.001;
	double ymin = y*0.001;
	double zmin = z*0.001;
	double xmax = (x+w)*0.001;
	double ymax = (y+h)*0.001;
	double zmax = (z+d)*0.001;

	cout << "\n" << endl;
	cout << r << "," << g << "," << b << endl; 
	cout << "x " << xmin << "," << xmax << endl;
	cout << "y " << ymin << "," << ymax << endl;
	cout << "z " << zmin << "," << zmax << endl;

	window_view->addCube(xmin,xmax,ymin,ymax,zmin,zmax,r,g,b,shapename);
	
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


