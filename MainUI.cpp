#include "MainUI.h"

#define POINTCLOUD_DIR "../pcd_files"
#define LISTCOLUMN 6


MainUI::MainUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainUI)
{
	cout << "MainUI::MainUI(QWidget *parent)" << endl;

    ui->setupUi(this);

	isRegisterCameraCallback = false;

	timerId_kinect = 0;

	isLoadPlaneParameter = false;
	
	//top menu
	///menu:viewer embedded
	connect(ui->action_connectkinect, SIGNAL(triggered()), this, SLOT(ButtonConnectPressed()));
	connect(ui->action_disconnectkinect, SIGNAL(triggered()), this, SLOT(ButtonDisconnectPressed()));
	connect(ui->action_loadpointcloud, SIGNAL(triggered()), this, SLOT(ButtonLoadPointCloudToViewerPressed()));
	connect(ui->action_savepointcloud, SIGNAL(triggered()), this, SLOT(ButtonSavePointCloudFromViewerPressed()));

	//menu:parameter
	connect(ui->actionLoad_Transformation, SIGNAL(triggered()), this, SLOT(ButtonLoadTransformParamPressed()));
	connect(ui->actionSave_Transformation, SIGNAL(triggered()), this, SLOT(ButtonSaveTransformParamPressed()));
	connect(ui->actionLoad_Passthrough, SIGNAL(triggered()), this, SLOT(ButtonLoadPassthroughParamPressed()));
	connect(ui->actionSave_Passthrough, SIGNAL(triggered()), this, SLOT(ButtonSavePassthroughParamPressed()));
	connect(ui->actionLoad_Camera, SIGNAL(triggered()), this, SLOT(ButtonLoadCameraParamPressed()));
	connect(ui->actionSave_Camera, SIGNAL(triggered()), this, SLOT(ButtonSaveCameraParamPressed()));
	connect(ui->actionLoad_Plane, SIGNAL(triggered()), this, SLOT(ButtonLoadPlaneParamPressed()));
	connect(ui->actionSave_Plane, SIGNAL(triggered()), this, SLOT(ButtonSavePlaneParamPressed()));

	//items above tab
	connect(ui->radio_axis_on, SIGNAL(clicked()), this, SLOT(RadioButtonAxisONSelected()));
	connect(ui->radio_axis_off, SIGNAL(clicked()), this, SLOT(RadioButtonAxisOFFSelected()));
	connect(ui->radio_boundingbox_on, SIGNAL(clicked()), this, SLOT(RadioButtonBBONSelected()));
	connect(ui->radio_boundingbox_off, SIGNAL(clicked()), this, SLOT(RadioButtonBBOFFSelected()));
	connect(ui->radio_boundingbox_aabb, SIGNAL(clicked()), this, SLOT(RadioButtonBBAABBSelected()));
	connect(ui->radio_boundingbox_obb, SIGNAL(clicked()), this, SLOT(RadioButtonBBOBBSelected()));
	connect(ui->bt_reloadcloud_to_viewer, SIGNAL(clicked()), this, SLOT(ButtonShowLoadedPointCloudPressed()));
	connect(ui->bt_clear_viewer_pointcloud, SIGNAL(clicked()), this, SLOT(ButtonClearViewerPointCloudPressed()));
	connect(ui->bt_clear_viewer_shape, SIGNAL(clicked()), this, SLOT(ButtonClearViewerShapePressed()));
	connect(ui->bt_undo_last_operation, SIGNAL(clicked()), this, SLOT(ButtonUndoLastedPointCloudPressed()));
	
	//tab:viewer parameter
	connect(ui->bt_apply_param_to_cam, SIGNAL(clicked()), this, SLOT(ButtonApplyParamtoCameraPressed()));
	connect(ui->bt_reset_viewerparam, SIGNAL(clicked()), this, SLOT(ButtonResetCamParamPressed()));
	connect(ui->bt_apply_camrot, SIGNAL(clicked()), this, SLOT(ButtonApplyCamRotationPressed()));
	connect(ui->bt_apply_campos, SIGNAL(clicked()), this, SLOT(ButtonApplyCamTranslationPressed()));

	//tab:cloud transformation
	connect(ui->bt_reset_cloudtransform, SIGNAL(clicked()), this, SLOT(ButtonResetCloudTransformationPressed()));
	connect(ui->bt_apply_cloudrot, SIGNAL(clicked()), this, SLOT(ButtonApplyCloudRotationPressed()));
	connect(ui->bt_apply_cloudtranslate, SIGNAL(clicked()), this, SLOT(ButtonApplyCloudTranslationPressed()));
	connect(ui->bt_reset_passthrough, SIGNAL(clicked()), this, SLOT(ButtonResetCloudPassthroughPressed()));
	connect(ui->bt_apply_passthrough, SIGNAL(clicked()), this, SLOT(ButtonApplyCloudPassthroughPressed()));
	connect(ui->bt_set_cloud_center, SIGNAL(clicked()), this, SLOT(ButtonSetCloudCenterPressed()));
	connect(ui->bt_set_cloud_corner, SIGNAL(clicked()), this, SLOT(ButtonSetCloudCornerPressed()));

	//tab:segmentation
	connect(ui->bt_apply_voxelgrid, SIGNAL(clicked()), this, SLOT(ButtonApplyVoxelGridPressed()));
	connect(ui->bt_apply_planesegment, SIGNAL(clicked()), this, SLOT(ButtonApplyPlaneSegmentPressed()));
	connect(ui->bt_get_plane_transform, SIGNAL(clicked()), this, SLOT(ButtonGetPlaneTransformPressed()));
	connect(ui->bt_apply_removeplane, SIGNAL(clicked()), this, SLOT(ButtonRemovePlanePressed()));
	connect(ui->bt_setplane_align_axis, SIGNAL(clicked()), this, SLOT(ButtonAlignPlaneToAxisCenterPressed()));
	connect(ui->bt_apply_outlierremove, SIGNAL(clicked()), this, SLOT(ButtonApplyOutlierPressed()));
	connect(ui->bt_show_cluster, SIGNAL(clicked()), this, SLOT(ButtonShowClusterPressed()));
	connect(ui->bt_extract_cluster, SIGNAL(clicked()), this, SLOT(ButtonExtractClusterPressed()));
	connect(ui->bt_show_cluster_bb, SIGNAL(clicked()), this, SLOT(ButtonShowClusterBBPressed()));

	//cluster list
	connect(ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(PressedTreeItem(QTreeWidgetItem *)));
	connect(ui->bt_item_load, SIGNAL(clicked()), this, SLOT(ButtonLoadPointCloudToListPressed()));
	connect(ui->bt_item_save, SIGNAL(clicked()), this, SLOT(ButtonSavePointCloudFromListPressed()));
	connect(ui->bt_all_load, SIGNAL(clicked()), this, SLOT(ButtonLoadAllItemPressed()));
	connect(ui->bt_all_save, SIGNAL(clicked()), this, SLOT(ButtonSaveAllItemPressed()));
	connect(ui->bt_item_remove, SIGNAL(clicked()), this, SLOT(ButtonRemoveItemPressed()));
	connect(ui->bt_item_clearall, SIGNAL(clicked()), this, SLOT(ButtonClearAllItemPressed()));
	connect(ui->bt_binpacking, SIGNAL(clicked()), this, SLOT(ButtonBinPackingPressed()));
	connect(ui->bt_show_packing, SIGNAL(clicked()), this, SLOT(ButtonShowPackingPressed()));
	connect(ui->bt_order_previous, SIGNAL(clicked()), this, SLOT(ButtonShowPrevPackingPressed()));
	connect(ui->bt_order_next, SIGNAL(clicked()), this, SLOT(ButtonShowNextPackingPressed()));
	connect(ui->bt_track_item_pos, SIGNAL(clicked()), this, SLOT(ButtonTrackItemPositionPressed()));

	
	//resize tree column size	
	ui->treeWidget->header()->resizeSection(0, 40);
	for (int j = 1; j < 5; j++)
	{
		ui->treeWidget->header()->resizeSection(j, 40);
	}
	for (int j = 5; j < 12; j++)
	{
		ui->treeWidget->header()->resizeSection(j, 80);
	}
	//myTreeWidget->headerView()->resizeSection(0 , 100);


}

MainUI::~MainUI()
{
    //delete ui;
	//delete program;
}

void MainUI::SetDataProcess(DataProcess* d) {dataprocess = d;}
void MainUI::SetViewerWindow(ViewerWindow* v) {viewerwindow = v;}

void MainUI::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void *stop_void)
{	//trigged when : mouse position is move in viewer area
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

	//check mouse click and scroll action
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		//cout << "LeftButton " << endl;
		SetCurrentCameraParameterToUI();

	}
	else if (event.getButton() == pcl::visualization::MouseEvent::RightButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		//cout << "RightButton " << endl;
		SetCurrentCameraParameterToUI();
	}
	else if (event.getButton() == pcl::visualization::MouseEvent::MouseScrollUp || event.getButton() == pcl::visualization::MouseEvent::MouseScrollDown)
	{
		//cout << "MiddleButton " << endl;
		SetCurrentCameraParameterToUI();
	}
}
void MainUI::SetCurrentCameraParameterToUI()
{

	vector<pcl::visualization::Camera> cam;
	viewerwindow->window_view->getCameras(cam);

	ui->in_focal_x->setText(QString::number(cam[0].focal[0]));
	ui->in_focal_y->setText(QString::number(cam[0].focal[1]));
	ui->in_focal_z->setText(QString::number(cam[0].focal[2]));

	ui->in_pos_x->setText(QString::number(cam[0].pos[0]));
	ui->in_pos_y->setText(QString::number(cam[0].pos[1]));
	ui->in_pos_z->setText(QString::number(cam[0].pos[2]));

	ui->in_up_x->setText(QString::number(cam[0].view[0]));
	ui->in_up_y->setText(QString::number(cam[0].view[1]));
	ui->in_up_z->setText(QString::number(cam[0].view[2]));

	ui->in_clipping_near->setText(QString::number(cam[0].clip[0]));
	ui->in_clipping_far->setText(QString::number(cam[0].clip[1]));
	ui->in_cam_angle_rad->setText(QString::number(cam[0].fovy));
	ui->in_cam_angle_deg->setText(QString::number(cam[0].fovy * 180 / M_PI));
}


//top menu
///menu:viewer embedded
void MainUI::ButtonConnectPressed()
{

	cout << "call ButtonConnectPressed()" << endl;
	if (!isRegisterCameraCallback)
	{
		viewerwindow->window_view->registerMouseCallback(&MainUI::mouseEventOccurred, *this, (void*)&viewerwindow->window_view);
		isRegisterCameraCallback = true;
	}

	if (timerId_kinect != 0)
	{
		QMessageBox::information(0, QString("Connect kinect"), QString("Kinect is already connected"), QMessageBox::Ok);
		return;
	}
	dataprocess->ConnectKinect();

	timerId_kinect = startTimer(100); //call timerEvent every 100 msec
}
void MainUI::timerEvent(QTimerEvent *event)
{
	dataprocess->ReadKinectInput();

	viewerwindow->UpdateWindowRGB(dataprocess->GetKinectRGBImage());

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetKinectPointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetKinectPointCloud());

	//ShowPointCloudSize();
}

void MainUI::ButtonDisconnectPressed()
{
	cout << "call ButtonDisconnectPressed()" << endl;

	if (timerId_kinect == 0)
	{
		QMessageBox::information(0, QString("Disconnect kinect"), QString("No kinect to disconnect"), QMessageBox::Ok);
		return;
	}
	killTimer(timerId_kinect);
	timerId_kinect = 0;

	dataprocess->DisconnectKinect();

	QMessageBox::information(0, QString("Disconnect kinect"), QString("Disconnect kinect complete"), QMessageBox::Ok);

}
void MainUI::ButtonSavePointCloudFromViewerPressed()
{
	cout << "call ButtonSavePointCloudFromViewerPressed()" << endl;

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save pointcloud data"), POINTCLOUD_DIR, tr("pointcloud (*.pcd)"));

	
	if (filename.trimmed().isEmpty()) // no file selected
	{
		cout << "no file selected" << endl;
		return;
	}
	else
	{
		cout << filename.toStdString() << endl;

		if (timerId_kinect != 0)//kinect is running
		{
			killTimer(timerId_kinect); //stop grabber data from kinect
			dataprocess->SavePointCloud(filename.toStdString(), dataprocess->GetKinectPointCloud());
			timerId_kinect = startTimer(100); // rerun kinect again
		}
		else
		{
			dataprocess->SavePointCloud(filename.toStdString(), dataprocess->GetCurrentDisplayPointCloud());
		}
		
	}

	
}
void MainUI::ButtonLoadPointCloudToViewerPressed()
{
	cout << "call ButtonLoadPointCloudToViewerPressed()" << endl;
	if (!isRegisterCameraCallback)
	{
		viewerwindow->window_view->registerMouseCallback(&MainUI::mouseEventOccurred, *this, (void*)&viewerwindow->window_view);
		isRegisterCameraCallback = true;
	}
	

	//disconnect kinect first
	if (timerId_kinect != 0)
	{
		ButtonDisconnectPressed();
	}

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Load pointcloud data"), POINTCLOUD_DIR, tr("pointcloud (*.pcd)"));
	if (filename.trimmed().isEmpty()) // no file selected
	{
		cout << "no file selected" << endl;
		return;
	}
	else
	{
		cout << filename.toStdString() << endl;
		dataprocess->LoadPointCloud(filename.toStdString());

		viewerwindow->UpdateWindowCloudViewer(dataprocess->GetLoadedPointCloud());
		//RadioButtonAxisONSelected();

		dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetLoadedPointCloud());
		cout << "GetCurrentDisplayPointCloudSize " << dataprocess->GetCurrentDisplayPointCloudSize() << endl;
	}
	
}

///menu:parameter
void MainUI::ButtonLoadTransformParamPressed()
{
	cout << "call ButtonLoadTransformParamPressed()" << endl;
}
void MainUI::ButtonSaveTransformParamPressed()
{
	cout << "call ButtonSaveTransformParamPressed()" << endl;
}
void MainUI::ButtonLoadPassthroughParamPressed()
{
	cout << "call ButtonLoadPassthroughParamPressed()" << endl;
}
void MainUI::ButtonSavePassthroughParamPressed()
{
	cout << "call ButtonSavePassthroughParamPressed()" << endl;
}
void MainUI::ButtonLoadCameraParamPressed()
{
	cout << "call ButtonLoadCameraParamPressed()" << endl;

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Open camera parameter file"), "../", tr("Text Files (*.txt)"));
	if (filename.trimmed().isEmpty()) // no file selected
	{
		return;
	}

	double focal_x, focal_y, focal_z;
	double pos_x, pos_y, pos_z;
	double up_x, up_y, up_z;
	double clipping_near, clipping_far;
	double cam_angle_rad;
	double cam_angle_deg;

	vector<pcl::visualization::Camera> cam;
	viewerwindow->window_view->getCameras(cam);

	dataprocess->cameraparam->ReadCameraParameter(filename.toStdString(),
		focal_x, focal_y, focal_z,
		pos_x, pos_y, pos_z,
		up_x, up_y, up_z,
		clipping_near, clipping_far,
		cam_angle_rad, cam_angle_deg);

	viewerwindow->SetCameraParameter(
		focal_x, focal_y, focal_z,
		pos_x, pos_y, pos_z,
		up_x, up_y, up_z,
		clipping_near, clipping_far,
		cam_angle_rad, cam_angle_deg);

	//QMessageBox::information(0, QString("/load camera parameter"), QString("Load camera parameter complete"), QMessageBox::Ok);


	
}
void MainUI::ButtonSaveCameraParamPressed()
{
	cout << "call ButtonSaveCameraParamPressed()" << endl;

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save camera parameter file"), "../", tr("Text Files (*.txt)"));
	if (filename.trimmed().isEmpty()) // no file selected
	{
		cout << "no file selected" << endl;
		return;
	}

	dataprocess->cameraparam->WriteCameraParameter(filename.toStdString(),
		ui->in_focal_x->text().toDouble(), 
		ui->in_focal_y->text().toDouble(),
		ui->in_focal_z->text().toDouble(),
		ui->in_pos_x->text().toDouble(), 
		ui->in_pos_y->text().toDouble(), 
		ui->in_pos_z->text().toDouble(),
		ui->in_up_x->text().toDouble(), 
		ui->in_up_y->text().toDouble(), 
		ui->in_up_z->text().toDouble(),
		ui->in_clipping_near->text().toDouble(), 
		ui->in_clipping_far->text().toDouble(),
		ui->in_cam_angle_rad->text().toDouble(), 
		ui->in_cam_angle_deg->text().toDouble());
	

	//QMessageBox::information(0, QString("Save camera parameter"), QString("Save camera parameter complete"), QMessageBox::Ok);

}
void MainUI::ButtonLoadPlaneParamPressed()
{
	cout << "call ButtonLoadPlaneParamPressed()" << endl;

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Open plane transform parameter"), "../", tr("Text Files (*.txt)"));
	if (filename.trimmed().isEmpty()) // no file selected
	{
		cout << "no file selected" << endl;
		return;
	}

	dataprocess->planeparam->ReadPlaneTransformParameter(
		filename.toStdString(),
		dataprocess->planeseg->transformextract->plane_coefficients_matrix,
		dataprocess->planeseg->transformextract->mass_center,
		dataprocess->planeseg->transformextract->major_vector,
		dataprocess->planeseg->transformextract->middle_vector,
		dataprocess->planeseg->transformextract->minor_vector,
		dataprocess->planeseg->transformextract->min_point_OBB,
		dataprocess->planeseg->transformextract->max_point_OBB,
		dataprocess->planeseg->transformextract->position_OBB,
		dataprocess->planeseg->transformextract->rotational_matrix_OBB);

	dataprocess->planeseg->SetHasPlaneTransformData(true);

	if (dataprocess->GetCurrentDisplayPointCloud()->size()>0)
	{

		cout << "dataprocess->GetCurrentDisplayPointCloud()->size()>0" << endl;

		dataprocess->planeseg->RemovePlaneOutside(dataprocess->GetCurrentDisplayPointCloud());
		viewerwindow->UpdateWindowCloudViewer(dataprocess->GetRemovedPlaneOutsidePointCloud());
		dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetRemovedPlaneOutsidePointCloud());

	}


	viewerwindow->AddBoundingBoxWindowCloudViewer(
		dataprocess->planeseg->transformextract->position_OBB,
		dataprocess->planeseg->transformextract->min_point_OBB,
		dataprocess->planeseg->transformextract->max_point_OBB,
		dataprocess->planeseg->transformextract->rotational_matrix_OBB,
		"planeseg OBB"
		);

	viewerwindow->AddVectorDirectionWindowCloudViewer(
		dataprocess->planeseg->transformextract->mass_center,
		dataprocess->planeseg->transformextract->major_vector,
		dataprocess->planeseg->transformextract->middle_vector,
		dataprocess->planeseg->transformextract->minor_vector,
		"planeseg ");


}
void MainUI::ButtonSavePlaneParamPressed()
{
	cout << "call ButtonSavePlaneParamPressed()" << endl;


	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save plane transform parameter"), "../", tr("Text Files (*.txt)"));
	if (filename.trimmed().isEmpty()) // no file selected
	{
		cout << "no file selected" << endl;
		return;
	}

	dataprocess->planeparam->WritePlaneTransformParameter(
		filename.toStdString(),
		dataprocess->planeseg->transformextract->plane_coefficients_matrix,
		dataprocess->planeseg->transformextract->mass_center,
		dataprocess->planeseg->transformextract->major_vector,
		dataprocess->planeseg->transformextract->middle_vector,
		dataprocess->planeseg->transformextract->minor_vector,
		dataprocess->planeseg->transformextract->min_point_OBB,
		dataprocess->planeseg->transformextract->max_point_OBB,
		dataprocess->planeseg->transformextract->position_OBB,
		dataprocess->planeseg->transformextract->rotational_matrix_OBB);
}

//items above tab
void MainUI::RadioButtonAxisONSelected()
{
	//cout << "call RadioButtonAxisONSelected()" << endl;
	viewerwindow->ToggleAxisONWindowCloudViewer();
	ui->radio_axis_on->setChecked(true);
	ui->radio_axis_off->setChecked(false);
}
void MainUI::RadioButtonAxisOFFSelected()
{
	//cout << "call RadioButtonAxisOFFSelected()" << endl;
	viewerwindow->ToggleAxisOFFWindowCloudViewer();
	ui->radio_axis_on->setChecked(false);
	ui->radio_axis_off->setChecked(true);
}
void MainUI::RadioButtonBBONSelected()
{
	cout << "call RadioButtonBBONSelected()" << endl;
}
void MainUI::RadioButtonBBOFFSelected()
{
	cout << "call RadioButtonBBOFFSelected()" << endl;
}	
void MainUI::RadioButtonBBAABBSelected()
{
	cout << "call RadioButtonBBAABBSelected()" << endl;
}
void MainUI::RadioButtonBBOBBSelected()
{
	cout << "call RadioButtonBBOBBSelected()" << endl;
}		
void MainUI::ButtonShowLoadedPointCloudPressed()
{
	cout << "call ButtonShowLoadedPointCloudPressed()" << endl;
}
void MainUI::ButtonClearViewerPointCloudPressed()
{
	cout << "call ButtonClearViewerPointCloudPressed()" << endl;
	viewerwindow->ClearPointCloudWindowCloudViewer();
	//viewerwindow->ClearShapeWindowCloudViewer();
	viewerwindow->ToggleAxisOFFWindowCloudViewer();

}	

void MainUI::ButtonClearViewerShapePressed()
{
	cout << "call ButtonClearViewerShapePressed()" << endl;
	viewerwindow->ClearShapeWindowCloudViewer();
}

void MainUI::ButtonUndoLastedPointCloudPressed()
{
	cout << "call ButtonUndoLastedPointCloudPressed()" << endl;
}

//tab:viewer parameter
void MainUI::ButtonApplyParamtoCameraPressed()
{
	cout << "call ButtonApplyParamtoCameraPressed()" << endl;
}
void MainUI::ButtonResetCamParamPressed()
{
	cout << "call ButtonResetCamParamPressed()" << endl;
}
void MainUI::ButtonApplyCamRotationPressed()
{
	cout << "call ButtonApplyCamRotationPressed()" << endl;
}
void MainUI::ButtonApplyCamTranslationPressed()
{
	cout << "call ButtonApplyCamTranslationPressed()" << endl;
}

//tab:cloud transformation
void MainUI::ButtonResetCloudTransformationPressed()
{
	cout << "call ButtonResetCloudTransformationPressed()" << endl;
}	
void MainUI::ButtonApplyCloudRotationPressed()
{
	cout << "call ButtonApplyCloudRotationPressed()" << endl;
}
void MainUI::ButtonApplyCloudTranslationPressed()
{
	cout << "call ButtonApplyCloudTranslationPressed()" << endl;
}
void MainUI::ButtonResetCloudPassthroughPressed()
{
	cout << "call ButtonResetCloudPassthroughPressed()" << endl;
}
void MainUI::ButtonApplyCloudPassthroughPressed()
{
	cout << "call ButtonApplyCloudPassthroughPressed()" << endl;
}
void MainUI::ButtonSetCloudCenterPressed()
{
	cout << "call ButtonSetCloudCenterPressed()" << endl;
}
void MainUI::ButtonSetCloudCornerPressed()
{
	cout << "call ButtonSetCloudCornerPressed()" << endl;
}

//tab:segmentation
void MainUI::ButtonApplyVoxelGridPressed()
{
	cout << "call ButtonApplyVoxelGridPressed()" << endl;
}
void MainUI::ButtonApplyPlaneSegmentPressed()
{
	cout << "call ButtonApplyPlaneSegmentPressed()" << endl;
	
	//disconnect kinect first
	if (timerId_kinect != 0)
	{
		ButtonDisconnectPressed();
	}

	double planethreshold = ui->in_plane_threshold->text().toDouble();
	dataprocess->planeseg->ApplyPlaneSegmentation(planethreshold, dataprocess->GetCurrentDisplayPointCloud());

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetAppliedRedPlanePointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetAppliedRedPlanePointCloud());

}

void MainUI::ButtonGetPlaneTransformPressed()
{
	cout << "call ButtonGetPlaneTransformPressed()" << endl;

	dataprocess->planeseg->CalculatePlaneTransformation(dataprocess->GetOnlyPlanePointCloud());
	dataprocess->planeseg->RemovePlaneOutside(dataprocess->GetCurrentDisplayPointCloud());

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetRemovedPlaneOutsidePointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetRemovedPlaneOutsidePointCloud());

	viewerwindow->AddBoundingBoxWindowCloudViewer(
		dataprocess->planeseg->transformextract->position_OBB,
		dataprocess->planeseg->transformextract->min_point_OBB,
		dataprocess->planeseg->transformextract->max_point_OBB,
		dataprocess->planeseg->transformextract->rotational_matrix_OBB,
		"planeseg OBB"
		);

	viewerwindow->AddVectorDirectionWindowCloudViewer(
		dataprocess->planeseg->transformextract->mass_center,
		dataprocess->planeseg->transformextract->major_vector,
		dataprocess->planeseg->transformextract->middle_vector,
		dataprocess->planeseg->transformextract->minor_vector,
		"planeseg ");

}

void MainUI::ButtonRemovePlanePressed()
{
	cout << "call ButtonRemovePlanePressed()" << endl;

	int pointcloudsize = dataprocess->GetRemovedPlanePointCloud()->size();

	if (pointcloudsize == 0)
	{
		cout << "no plane segmentation pointcloud data" << endl;
		return;
	}
	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetRemovedPlanePointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetRemovedPlanePointCloud());



}
void MainUI::ButtonAlignPlaneToAxisCenterPressed()
{
	cout << "call ButtonAlignPlaneToAxisCenterPressed()" << endl;

	PointTypeXYZRGB target_pos;
	target_pos.x = 0;
	target_pos.y = 0;
	target_pos.z = 0;

	dataprocess->MovePointCloudFromTo(dataprocess->GetCurrentDisplayPointCloud(),
		dataprocess->planeseg->transformextract->position_OBB,
		target_pos);

	Eigen::Matrix<float, 1, 3>  floor_plane_normal_vector, target_plane_normal_vector;
	//floor_plane_normal_vector[0] = dataprocess->planeseg->transformextract->plane_coefficients_matrix(0);
	//floor_plane_normal_vector[1] = dataprocess->planeseg->transformextract->plane_coefficients_matrix(1);
	//floor_plane_normal_vector[2] = dataprocess->planeseg->transformextract->plane_coefficients_matrix(2);

	floor_plane_normal_vector[0] = dataprocess->planeseg->transformextract->minor_vector(0);
	floor_plane_normal_vector[1] = dataprocess->planeseg->transformextract->minor_vector(1);
	floor_plane_normal_vector[2] = dataprocess->planeseg->transformextract->minor_vector(2);

	target_plane_normal_vector[0] = 0.0;
	target_plane_normal_vector[1] = 1.0;//xz plane
	target_plane_normal_vector[2] = 0.0;

	dataprocess->RotatePointCloudAtAxis(dataprocess->GetCurrentDisplayPointCloud(),
		floor_plane_normal_vector, target_plane_normal_vector);


	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetCurrentDisplayPointCloud());
	viewerwindow->ClearShapeWindowCloudViewer();

}
void MainUI::ButtonApplyOutlierPressed()
{
	cout << "call ButtonApplyOutlierPressed()" << endl;

	dataprocess->StoreLastedOperationCloud(dataprocess->GetCurrentDisplayPointCloud());

	dataprocess->outlierremove->StatisticalOutlierRemoval(
		dataprocess->GetCurrentDisplayPointCloud(), 
		ui->in_outlier_meank->text().toInt(),
		ui->in_outlier_stddev->text().toInt());

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetCurrentDisplayPointCloud());

}
void MainUI::ButtonShowClusterPressed()
{
	cout << "call ButtonShowClusterPressed()" << endl;

	int pointcloudsize = dataprocess->GetCurrentDisplayPointCloud()->size();

	if (pointcloudsize == 0)
	{
		cout << "no point cloud" << endl;
		return;
	}

	double cluster_tolerance = ui->in_clusterextract_tolerance->text().toDouble();
	int cluster_min_percentage = ui->in_clusterextract_min->text().toInt();
	int cluster_max_percentage = ui->in_clusterextract_max->text().toInt();
	int cluster_min_size = cluster_min_percentage*pointcloudsize / 100;
	int cluster_max_size = cluster_max_percentage*pointcloudsize / 100;


	dataprocess->clusterextract->SetClusterExtractValue(cluster_tolerance, cluster_min_size, cluster_max_size);
	dataprocess->clusterextract->ExtractCluster(dataprocess->GetCurrentDisplayPointCloud());

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetColoredClusterPointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetColoredClusterPointCloud());
	//dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetRemovedPlaneOutsidePointCloud());



}
inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v) 
{ 
	pcl::PointXYZRGB p; 
	p.x = v[0]; p.y = v[1]; p.z = v[2]; 
	return p; 
}
void MainUI::ShowMinMaxCenterPoint(ObjectTransformationData *obj, string id_name)
{
	//position_OBB
	viewerwindow->AddSphereWindowCloudViewer(obj->transform->position_OBB, 0.01,
		1.0, 0, 0, " obb" + id_name);
	viewerwindow->AddTextWindowCloudViewer(obj->transform->position_OBB, 0.01,
		1.0, 1.0, 1.0, "obb" + id_name, " text obb" + id_name);

	//mass_center_point
	viewerwindow->AddSphereWindowCloudViewer(obj->transform->mass_center_point, 0.01,
		0, 1.0, 0, " mass" + id_name);
	viewerwindow->AddTextWindowCloudViewer(obj->transform->mass_center_point, 0.01,
		1.0, 1.0, 1.0, "mass" + id_name, " text mass" + id_name);
	
	//min_point_OBB
	viewerwindow->AddSphereWindowCloudViewer(obj->transform->min3d_point, 0.01,
		0, 0, 1.0, " min"+ id_name);
	viewerwindow->AddTextWindowCloudViewer(obj->transform->min3d_point, 0.01,
		1.0, 1.0, 1.0, "min"+ id_name,  "text min"+ id_name);

	//max_point_OBB
	viewerwindow->AddSphereWindowCloudViewer(obj->transform->max3d_point, 0.01,
		0, 0, 1.0, " max"+ id_name);
	viewerwindow->AddTextWindowCloudViewer(obj->transform->max3d_point, 0.01,
		1.0, 1.0, 1.0, "max"+ id_name, " text max"+ id_name);

}
void MainUI::ButtonExtractClusterPressed()
{
	cout << "call ButtonExtractClusterPressed()" << endl;

	dataprocess->SeparateContainerAndItems(dataprocess->clusterextract->GetExtractCluster());
	
	dataprocess->CalculateContainerTransformation();
	ShowMinMaxCenterPoint(dataprocess->container, "container");
	ui->in_bin_w->setText(QString::number(dataprocess->container->width));
	ui->in_bin_d->setText(QString::number(dataprocess->container->depth));
	ui->in_bin_h->setText(QString::number(dataprocess->container->height));

	dataprocess->CalculateItemsTransformation();
	for (int i = 0; i < dataprocess->items.size();i++)
	{
		ShowMinMaxCenterPoint(dataprocess->items[i], "item_" + to_string(i+1));

		QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);

		item->setText(0, QString::number(i+1));
		item->setText(1, QString::number(dataprocess->items[i]->width));
		item->setText(2, QString::number(dataprocess->items[i]->depth));
		item->setText(3, QString::number(dataprocess->items[i]->height));
		item->setText(4, QString::number(dataprocess->items[i]->object_pointcloud->size()));
		item->setText(5, QString::number(dataprocess->items[i]->transform->position_OBB.x));
		item->setText(6, QString::number(dataprocess->items[i]->transform->position_OBB.y));
		item->setText(7, QString::number(dataprocess->items[i]->transform->position_OBB.z));


		item->setTextAlignment(0, Qt::AlignHCenter);
		for (int j = 1; j < 5; j++)
		{
			item->setTextAlignment(j, Qt::AlignRight);
		}
		for (int j = 5; j < 12; j++)
		{
			item->setTextAlignment(j, Qt::AlignLeft);
		}

		item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);

		ui->treeWidget->addTopLevelItem(item);

	}
}

void::MainUI::ButtonShowClusterBBPressed()
{
	// draw boundingbox+vector
	viewerwindow->AddBoundingBoxWindowCloudViewer(
		dataprocess->container->transform->position_OBB,
		dataprocess->container->transform->min_point_OBB,
		dataprocess->container->transform->max_point_OBB,
		dataprocess->container->transform->rotational_matrix_OBB,
		"container OBB"
		);

	viewerwindow->AddVectorDirectionWindowCloudViewer(
		dataprocess->container->transform->mass_center,
		dataprocess->container->transform->major_vector,
		dataprocess->container->transform->middle_vector,
		dataprocess->container->transform->minor_vector,
		"container vector ");



	for (int i = 0; i < dataprocess->items.size(); i++)
	{
		viewerwindow->AddBoundingBoxWindowCloudViewer(
			dataprocess->items[i]->transform->position_OBB,
			dataprocess->items[i]->transform->min_point_OBB,
			dataprocess->items[i]->transform->max_point_OBB,
			dataprocess->items[i]->transform->rotational_matrix_OBB,
			"items OBB " + i);

		viewerwindow->AddVectorDirectionWindowCloudViewer(
			dataprocess->items[i]->transform->mass_center,
			dataprocess->items[i]->transform->major_vector,
			dataprocess->items[i]->transform->middle_vector,
			dataprocess->items[i]->transform->minor_vector,
			"items vector " + i);
			
	}



}


//cluster list
void MainUI::PressedTreeItem(QTreeWidgetItem *current_select_item)
{
	cout << "call PressedTreeItem()" << endl;
};
void MainUI::ButtonLoadPointCloudToListPressed()
{
	cout << "call ButtonLoadPointCloudToListPressed()" << endl;
}
void MainUI::ButtonSavePointCloudFromListPressed()
{
	cout << "call ButtonSavePointCloudFromListPressed()" << endl;
}
void MainUI::ButtonLoadAllItemPressed()
{
	cout << "call ButtonLoadAllItemPressed()" << endl;
}
void MainUI::ButtonSaveAllItemPressed()
{
	cout << "call ButtonSaveAllItemPressed()" << endl;
}
void MainUI::ButtonRemoveItemPressed()
{
	cout << "call ButtonRemoveItemPressed()" << endl;
}
void MainUI::ButtonClearAllItemPressed()
{
	cout << "call ButtonClearAllItemPressed()" << endl;
}
void MainUI::ButtonBinPackingPressed()
{
	cout << "call ButtonBinPackingPressed()" << endl;

	int total_boxes = ui->treeWidget->topLevelItemCount();

	int *boxes_x_orient = new int[total_boxes];
	int *boxes_y_orient = new int[total_boxes];
	int *boxes_z_orient = new int[total_boxes];
	int *boxes_bin_num = new int[total_boxes];

	int *boxes_x_pos = new int[total_boxes];
	int *boxes_y_pos = new int[total_boxes];
	int *boxes_z_pos = new int[total_boxes];
	int *boxes_item_num = new int[total_boxes];

	int *boxes_w, *boxes_h, *boxes_d;

	int *boxes_width = new int[total_boxes];
	int *boxes_height = new int[total_boxes];
	int *boxes_depth = new int[total_boxes];

	for (int i = 0; i < total_boxes; i++)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		boxes_width[i] = item->text(1).toInt();
		boxes_height[i] = item->text(2).toInt();
		boxes_depth[i] = item->text(3).toInt();
	}

	boxes_w = boxes_width;
	boxes_h = boxes_height;
	boxes_d = boxes_depth;

	for (int i = 0; i < total_boxes; ++i)
	{
		boxes_x_orient[i] = 0;
		boxes_y_orient[i] = 0;
		boxes_z_orient[i] = 0;
		boxes_bin_num[i] = 0;

		boxes_x_pos[i] = 0;
		boxes_y_pos[i] = 0;
		boxes_z_pos[i] = 0;
		boxes_item_num[i] = 0;
	}


	binpack = new CalculateBppErhan();
	binpack->CalculateBinpack(
		total_boxes,
		ui->in_bin_w->text().toDouble(), 
		ui->in_bin_d->text().toDouble(), 
		ui->in_bin_h->text().toDouble(),
		boxes_w, boxes_h, boxes_d,
		boxes_x_pos, boxes_y_pos, boxes_z_pos,
		boxes_x_orient, boxes_y_orient, boxes_z_orient,
		boxes_bin_num, boxes_item_num);

	// check if use bin >1
	int item_not_fit = 0;
	for (int i = 0; i < total_boxes; i++)
	{
		/*std::cout
		<< i << ":"
		<< boxes_width[i] << " " << boxes_height[i] << " " << boxes_depth[i] << " "
		<< "bin_num:" << boxes_bin_num[i] << " "
		<< "pos:"
		<< boxes_x_pos[i] << " " << boxes_y_pos[i] << " " << boxes_z_pos[i] << " "
		<< std::endl;
		*/
		if (boxes_bin_num[i] != 1) item_not_fit++;

	}
	if (item_not_fit != 0)
	{
		//std::cout << " cannot fit another " << item_not_fit << " boxes to bin" << std::endl;
		QString text = "Another " + QString::number(item_not_fit) + " boxes cannot fit in to bin"; // CORRECT
		QMessageBox::information(0, QString("Cannot fit all boxes"), text, QMessageBox::Ok);
	}
	//remove remain boxes

	//binpack->SortBoxesOrder();

	viewerwindow->ShowBinpacking(
		total_boxes,
		ui->in_bin_w->text().toDouble(),
		ui->in_bin_d->text().toDouble(),
		ui->in_bin_h->text().toDouble(),
		boxes_x_orient, boxes_y_orient, boxes_z_orient,
		boxes_x_pos, boxes_y_pos, boxes_z_pos);



}
void MainUI::ButtonShowPackingPressed()
{
	cout << "call ButtonShowPackingPressed()" << endl;
}
void MainUI::ButtonShowPrevPackingPressed()
{
	cout << "call ButtonShowPrevPackingPressed()" << endl;
}
void MainUI::ButtonShowNextPackingPressed()
{
	cout << "call ButtonShowNextPackingPressed()" << endl;
}
void MainUI::ButtonTrackItemPositionPressed()
{
	cout << "call ButtonTrackItemPositionPressed()" << endl;

	double red = 1.0;
	double green = 1.0;
	double blue = 1.0;

	for (int i = 0; i < dataprocess->items.size(); i++)
	{
		/*viewerwindow->AddTextWindowCloudViewer(
			dataprocess->items[i].transform->min_point_OBB,
			dataprocess->items[i].transform->major_vector,
			red, green, blue, "min",
			"items min text " + i);

		viewerwindow->AddTextWindowCloudViewer(
			dataprocess->items[i].transform->max_point_OBB,
			dataprocess->items[i].transform->major_vector,
			red, green, blue, "max",
			"items max text " + i);
			*/

		viewerwindow->AddSymbolWindowCloudViewer(
			dataprocess->items[i]->transform->position_OBB,
			dataprocess->items[i]->transform->min3d_point,
			dataprocess->items[i]->transform->max3d_point,
			dataprocess->items[i]->transform->mass_center,
			dataprocess->items[i]->transform->major_vector,
			dataprocess->items[i]->transform->middle_vector,
			red, green, blue, "symbol " + i);

		
	}
}


