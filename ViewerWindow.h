#ifndef VIEWERWINDOW_H
#define VIEWERWINDOW_H
#include "DataProcess.h"
#include "SharedHeader.h"

class ViewerWindow : public QObject
{
	Q_OBJECT

private:
	DataProcess *dataprocess;


	void timerEvent(QTimerEvent *event);
	//QTime time_animate;
	int timer_animate;

	//transform.translation() << translate_x, translate_y, translate_z;
	//pcl::transformPointCloud(*cloud, *cloud, transform);
	PointTypeXYZRGB current_animate_cube_pos, target_animate_cube_pos;
	float cube_x_dim, cube_y_dim, cube_z_dim;
	PointTypeXYZRGB input_cube_dim;
	PointTypeXYZRGB cube_translate_diff; 

	//transform.rotate(Eigen::AngleAxisf(theta, rotation_vector));
	//pcl::transformPointCloud(*cloud, *cloud, transform);
	float current_theta_cube_rot;
	//float target_theta_cube_rot;
	//float target_cube_rotation_vector;
	Eigen::Matrix<float, 1, 3>  cube_first_rotate_vector;
	Eigen::Matrix<float, 1, 3>  cube_second_rotate_vector;
	Eigen::Matrix<float, 1, 3>  cube_adjust_translate;

	float cube_x_dif, cube_z_dif, cube_theta_dif;
	int translate_count,rotate_count;


	bool first_translate, second_rotate, third_rotate,adjust_translate;
	bool first_translate_done, second_rotate_done, third_rotate_done, adjust_translate_done;

	pcl::PolygonMesh current_animate_cube;

public:
	//Addxxxfuntion() => did not spin a window, just Add!
	boost::shared_ptr<pcl::visualization::PCLVisualizer> window_view;

    ViewerWindow();
	void SetDataProcess(DataProcess* d);

	void SetCameraParameter(
		double focal_x, double focal_y, double focal_z,
		double pos_x, double pos_y, double pos_z,
		double up_x, double up_y, double up_z,
		double clipping_near, double clipping_far,
		double cam_angle_rad, double cam_angle_deg);

	void UpdateWindowCloudViewer(PointCloudXYZRGB::Ptr pointcloud);
	void UpdateWindowRGB(cv::Mat image);

	void ClearPointCloudWindowCloudViewer();
	void ClearShapeWindowCloudViewer();	

	void ToggleAxisONWindowCloudViewer();
	void ToggleAxisOFFWindowCloudViewer();

	void randomcolorfloat(float &r, float &g, float &b);
	void randomcolorint(int &r, int &g, int &b);

	void AddArrowObj();
	void AddArrowPolygonMesh(
		float x, float y, float z,
		float r, float g, float b);

	void AddPlanarAtOrigin(double plane_halflegth_x, double plane_halflegth_z,
		double r, double g, double b, string planename);

	void AddBoundingBoxWindowCloudViewer(PointTypeXYZRGB position_OBB,
		PointTypeXYZRGB min_point_OBB, PointTypeXYZRGB max_point_OBB,
		Eigen::Matrix3f rotational_matrix_OBB, string cloudname);
	
	void AddVectorDirectionWindowCloudViewer(Eigen::Vector3f mass_center,
		Eigen::Vector3f major_vector, Eigen::Vector3f middle_vector, Eigen::Vector3f minor_vector,
		string cloudname);
	
	void AddTextWindowCloudViewer(PointTypeXYZRGB point_position, double text_scale,
		double r, double g, double b, string drawtext, string cloudname);

	void AddCircleWindowCloudViewer(
		PointTypeXYZRGB position_OBB, float radius,
		double r, double g, double b, string cloudname);
	
	void AddSphereWindowCloudViewer(PointTypeXYZRGB point_position, double radius, double r, double g, double b, string id_name);

	void AddSymbolWindowCloudViewer(
		PointTypeXYZRGB position_OBB,
		PointTypeXYZRGB min3d,
		PointTypeXYZRGB max3d,
		Eigen::Vector3f mass_center,
		Eigen::Vector3f major_vector,
		Eigen::Vector3f minor_vector,
		double r, double g, double b, string cloudname);


	pcl::PolygonMesh visualizerGetCameraMesh(const Eigen::Matrix3f& R, const Eigen::Vector3f& t, 
		float r, float g, float b,
		Eigen::Vector3f& vforward, Eigen::Vector3f& rgb);

	PointTypeXYZRGB AddItemArrowDirectionSymbol(
		float w, float h, float d, 
		float x, float y, float z, 
		float r, float g, float b,
		string symbolname);


	void AddItemCube(float w, float h, float d,
		float x, float y, float z,
		float r, float g, float b,
		string shapename);

	
	void AddItemCubeShader(
		float x_dim, float y_dim, float z_dim,
		float target_x, float target_y, float target_z,
		Eigen::Matrix<float, 1, 3> rotation_matrix, float rotate_degree,
		int r, int g, int b,
		string shapename);

	pcl::PolygonMesh CreteNewCubePolymesh(
		float x_dim, float y_dim, float z_dim, 
		int r, int g, int b);

	pcl::PolygonMesh CreteNewCubePolymeshAtCentroid(
		float x_dim, float y_dim, float z_dim,
		int r, int g, int b);

	pcl::PolygonMesh TransformItemCubeShaderAtCenttroid(
		pcl::PolygonMesh cube_mesh,
		float translate_x, float translate_y, float translate_z,
		Eigen::Matrix<float, 1, 3> rotation_matrix, float rotate_degree,
		string shapename);





	void ShowBinPackingTarget(ObjectTransformationData *container, ObjectTransformationData* item, int i);
	void ShowBinpackingIndication(ObjectTransformationData *container, ObjectTransformationData* item, int i);
	void ShowBinpackingAnimation(ObjectTransformationData *container, ObjectTransformationData* item);



};

#endif // VIEWERWINDOW_H
