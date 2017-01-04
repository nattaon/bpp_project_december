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
	last_select_item_index = -1;

	isLoadPlaneParameter = false;

	viewerembeded = new ViewerEmbeded(ui->widget);

	
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
	connect(ui->bt_apply_zero_cloudrot, SIGNAL(clicked()), this, SLOT(ButtonApplyZeroCloudRotationPressed()));
	connect(ui->bt_apply_zero_cloudtranslate, SIGNAL(clicked()), this, SLOT(ButtonApplyZeroCloudTranslationPressed()));

	
	connect(ui->bt_reset_passthrough, SIGNAL(clicked()), this, SLOT(ButtonResetCloudPassthroughPressed()));
	connect(ui->bt_apply_passthrough, SIGNAL(clicked()), this, SLOT(ButtonApplyCloudPassthroughPressed()));
	connect(ui->bt_set_cloud_center, SIGNAL(clicked()), this, SLOT(ButtonSetCloudCenterPressed()));
	connect(ui->bt_set_cloud_corner, SIGNAL(clicked()), this, SLOT(ButtonSetCloudCornerPressed()));
	connect(ui->bt_set_cloud_align_corner, SIGNAL(clicked()), this, SLOT(ButtonSetCloudAlignCornerPressed()));
	connect(ui->bt_calculate_cloud_transform, SIGNAL(clicked()), this, SLOT(ButtonCalculateCloudTransformPressed()));

	//tab:segmentation
	connect(ui->bt_apply_voxelgrid, SIGNAL(clicked()), this, SLOT(ButtonApplyVoxelGridPressed()));
	connect(ui->bt_apply_planesegment, SIGNAL(clicked()), this, SLOT(ButtonApplyPlaneSegmentPressed()));
	connect(ui->bt_get_plane_transform, SIGNAL(clicked()), this, SLOT(ButtonCalculatePlaneTransformPressed()));
	connect(ui->bt_apply_removeplane, SIGNAL(clicked()), this, SLOT(ButtonRemovePlanePressed()));
	connect(ui->bt_setplane_align_axis, SIGNAL(clicked()), this, SLOT(ButtonAlignPlaneToAxisCenterPressed()));
	
	connect(ui->bt_load_plane_txt, SIGNAL(clicked()), this, SLOT(ButtonLoadPlaneParamPressed()));
	connect(ui->bt_save_plane_txt, SIGNAL(clicked()), this, SLOT(ButtonSavePlaneParamPressed()));
	
	connect(ui->bt_apply_outlierremove, SIGNAL(clicked()), this, SLOT(ButtonApplyOutlierPressed()));
	connect(ui->bt_show_cluster, SIGNAL(clicked()), this, SLOT(ButtonShowClusterPressed()));
	connect(ui->bt_extract_cluster, SIGNAL(clicked()), this, SLOT(ButtonExtractClusterPressed()));
	connect(ui->bt_show_cluster_bb, SIGNAL(clicked()), this, SLOT(ButtonShowClusterBBPressed()));
	connect(ui->bt_show_cluster_vector, SIGNAL(clicked()), this, SLOT(ButtonShowClusterVectorPressed()));

	//cluster list
	connect(ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(PressedTreeItem(QTreeWidgetItem *)));
	connect(ui->bt_item_load, SIGNAL(clicked()), this, SLOT(ButtonLoadPointCloudToListPressed()));
	connect(ui->bt_item_save, SIGNAL(clicked()), this, SLOT(ButtonSavePointCloudFromListPressed()));
	
	connect(ui->bt_align_all_items_to_axis, SIGNAL(clicked()), this, SLOT(ButtonAlignAllItemAxisPressed()));
	connect(ui->bt_save_all_items_to_pcd, SIGNAL(clicked()), this, SLOT(ButtonSaveAllItemPcdPressed()));

	connect(ui->bt_all_load, SIGNAL(clicked()), this, SLOT(ButtonLoadAllItemPressed()));
	connect(ui->bt_all_save, SIGNAL(clicked()), this, SLOT(ButtonSaveAllItemPressed()));

	connect(ui->bt_item_remove, SIGNAL(clicked()), this, SLOT(ButtonRemoveItemPressed()));
	connect(ui->bt_item_clearall, SIGNAL(clicked()), this, SLOT(ButtonClearAllItemPressed()));

	connect(ui->bt_binpacking, SIGNAL(clicked()), this, SLOT(ButtonCalculateBinPackingPressed()));
	connect(ui->bt_track_item_pos, SIGNAL(clicked()), this, SLOT(ButtonTrackItemPositionPressed()));

	connect(ui->bt_show_packing_target, SIGNAL(clicked()), this, SLOT(ButtonShowPackingTargetPressed()));
	connect(ui->bt_show_packing_indicate, SIGNAL(clicked()), this, SLOT(ButtonShowPackingIndicatePressed()));
	connect(ui->bt_show_packing_animation, SIGNAL(clicked()), this, SLOT(ButtonShowPackingAnimationPressed()));

	connect(ui->bt_order_one, SIGNAL(clicked()), this, SLOT(ButtonShowZeroPackingPressed()));
	connect(ui->bt_order_previous, SIGNAL(clicked()), this, SLOT(ButtonShowPrevPackingPressed()));
	connect(ui->bt_order_next, SIGNAL(clicked()), this, SLOT(ButtonShowNextPackingPressed()));

	connect(ui->bt_save_packing_info, SIGNAL(clicked()), this, SLOT(ButtonSaveBinPackingInfoPressed()));
	connect(ui->bt_load_packing_info, SIGNAL(clicked()), this, SLOT(ButtonLoadBinPackingInfoPressed()));
	
	//resize tree column size	

	for (int j = 0; j < 5; j++)
	{
		ui->treeWidget->header()->resizeSection(j, 45);
	}

	for (int j = 5; j < 11; j++)
	{
		ui->treeWidget->header()->resizeSection(j, 80);
	}
	ui->treeWidget->header()->resizeSection(11, 500);

	//QApplication::instance()->installEventFilter(this);
	
}

MainUI::~MainUI()
{
    //delete ui;
	//delete program;
}

void MainUI::SetDataProcess(DataProcess* d) {dataprocess = d;}
void MainUI::SetViewerWindow(ViewerWindow* v) {viewerwindow = v;}

bool MainUI::eventFilter(QObject *object, QEvent *event) 
{
	if (event->type() == QEvent::KeyPress) {
		QKeyEvent* key_event = static_cast<QKeyEvent*>(event);
		cout << "key " << key_event->key() << " object:" << object->objectName().toStdString() << endl;


		if (key_event->key() == Qt::Key_Right)
		{
			//PressedNextOrder();
		}
		else if (key_event->key() == Qt::Key_Left)
		{
			//PressedPreviousOrder();
		}
	}
	return false;
}
void MainUI::keyPressEvent(QKeyEvent * event)
{
	//after opengl show boxes, this function is not working tooo!!!!
	//solve: set ui as nofocus
	cout << "MainWindow event->key() " << event->key() << endl;

	if (event->key() == Qt::Key_Up)
	{
		if (last_select_item_index-1>=0)
		{
			cout << "up   : select index " << last_select_item_index - 1 << endl;
			QTreeWidgetItem* item = ui->treeWidget->topLevelItem(last_select_item_index-1);
			ui->treeWidget->setCurrentItem(item);
			PressedTreeItem(item);
		}
	}
	else if (event->key() == Qt::Key_Down)
	{
		if (last_select_item_index + 1 < ui->treeWidget->topLevelItemCount())
		{
			cout << "down : select index " << last_select_item_index +1 << endl;
			QTreeWidgetItem* item = ui->treeWidget->topLevelItem(last_select_item_index+1);
			ui->treeWidget->setCurrentItem(item);
			PressedTreeItem(item);
		}
	}
	else if (event->key() == Qt::Key_Left)
	{
		
	}
	else if (event->key() == Qt::Key_Right)
	{
		
	}

	QWidget::keyPressEvent(event);

}
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

	dataprocess->ApplyPassthroughFilter(
		dataprocess->GetKinectPointCloud(),
		ui->edit_passthrough_xmin->text().toDouble(),
		ui->edit_passthrough_xmax->text().toDouble(),
		ui->edit_passthrough_ymin->text().toDouble(),
		ui->edit_passthrough_ymax->text().toDouble(),
		ui->edit_passthrough_zmin->text().toDouble(),
		ui->edit_passthrough_zmax->text().toDouble());


	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetKinectPointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetKinectPointCloud());

	//ShowPointCloudSize();
}

void MainUI::ButtonDisconnectPressed()
{
	cout << "call ButtonDisconnectPressed()" << endl;
	cout << "timerId_kinect=" << timerId_kinect << endl;
	if (timerId_kinect == 0)
	{
		QMessageBox::information(0, QString("Disconnect kinect"), QString("No kinect to disconnect"), QMessageBox::Ok);
		return;
	}
	killTimer(timerId_kinect);
	timerId_kinect = 0;

	dataprocess->DisconnectKinect();

	//QMessageBox::information(0, QString("Disconnect kinect complete"), 
	//	QString("Disconnect kinect complete"), QMessageBox::Ok);
	// this message box auto close...
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
		ui->text_cloudname->setText(filename);


		cout << filename.toStdString() << endl;
		dataprocess->LoadPointCloud(filename.toStdString());

		viewerwindow->UpdateWindowCloudViewer(dataprocess->GetLoadedPointCloud());
		//RadioButtonAxisONSelected();

		dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetLoadedPointCloud());
		cout << "GetCurrentDisplayPointCloudSize " << dataprocess->GetCurrentDisplayPointCloudSize() << endl;

		RadioButtonAxisONSelected();
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
	WritePlaneParamToUI();

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
	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetLastedOperateDisplayPointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetLastedOperateDisplayPointCloud());

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
void MainUI::ButtonApplyZeroCloudRotationPressed()
{
	ui->edit_cloud_rot_x->setText("0");
	ui->edit_cloud_rot_y->setText("0");
	ui->edit_cloud_rot_z->setText("0");

}
void MainUI::ButtonApplyZeroCloudTranslationPressed()
{
	ui->edit_cloud_tran_x->setText("0.0");
	ui->edit_cloud_tran_y->setText("0.0");
	ui->edit_cloud_tran_z->setText("0.0");
}

void MainUI::ButtonApplyCloudRotationPressed()
{
	cout << "call ButtonApplyCloudRotationPressed()" << endl;
	
	PointCloudXYZRGB::Ptr pointcloud;
	
	// rotate cloud at viewer window
	if (last_select_item_index == -1)
	{
		pointcloud = dataprocess->GetCurrentDisplayPointCloud();

	}
	else // rotate cloud at viewer embeded
	{
		pointcloud = dataprocess->items[last_select_item_index]->object_pointcloud;
	}

	if (pointcloud->size() <= 0)
	{
		QMessageBox::information(0, QString("ButtonApplyCloudRotationPressed"),
			QString("No Item selected"), QMessageBox::Ok);

	}

	

	float rotate_x = ui->edit_cloud_rot_x->text().toDouble();
	float rotate_y = ui->edit_cloud_rot_y->text().toDouble();
	float rotate_z = ui->edit_cloud_rot_z->text().toDouble();
	float rotate_degree;
	Eigen::Matrix<float, 1, 3>  rotation_vector;

	if (rotate_x != 0.0)
	{
		rotate_degree = rotate_x;

		rotation_vector[0] = 1.0;
		rotation_vector[1] = 0.0;
		rotation_vector[2] = 0.0;
	}
	else if (rotate_y != 0.0)
	{
		rotate_degree = rotate_y;

		rotation_vector[0] = 0.0;
		rotation_vector[1] = 1.0;
		rotation_vector[2] = 0.0;
	}
	else if (rotate_z != 0.0)
	{
		rotate_degree = rotate_z;

		rotation_vector[0] = 0.0;
		rotation_vector[1] = 0.0;
		rotation_vector[2] = 1.0;
	}
	else
	{
		cout << endl << "ERROR ROTATE!!!" << endl;

	}

	dataprocess->RotatePointCloud(pointcloud,
		rotate_degree, rotation_vector);

	// rotate cloud at viewer window
	if (last_select_item_index == -1)
	{
		viewerwindow->UpdateWindowCloudViewer(pointcloud);
	}
	else // rotate cloud at viewer embeded
	{
		viewerembeded->UpdateCloudViewer(pointcloud);
	}
	

}
void MainUI::ButtonApplyCloudTranslationPressed()
{
	cout << "call ButtonApplyCloudTranslationPressed()" << endl;

	PointCloudXYZRGB::Ptr pointcloud;

	// translate cloud at viewer window
	if (last_select_item_index == -1)
	{
		pointcloud = dataprocess->GetCurrentDisplayPointCloud();

	}
	else // translate cloud at viewer embeded
	{
		pointcloud = dataprocess->items[last_select_item_index]->object_pointcloud;
	}

	if (pointcloud->size() <= 0)
	{
		QMessageBox::information(0, QString("ButtonApplyCloudRotationPressed"),
			QString("No Item selected"), QMessageBox::Ok);

	}

	float translate_x = ui->edit_cloud_tran_x->text().toDouble();
	float translate_y = ui->edit_cloud_tran_y->text().toDouble();
	float translate_z = ui->edit_cloud_tran_z->text().toDouble();

	dataprocess->TranslatePointCloud(pointcloud,
		translate_x, translate_y, translate_z);

	// rotate cloud at viewer window
	if (last_select_item_index == -1)
	{
		viewerwindow->UpdateWindowCloudViewer(pointcloud);
	}
	else // rotate cloud at viewer embeded
	{
		viewerembeded->UpdateCloudViewer(pointcloud);
	}
}
void MainUI::ButtonResetCloudPassthroughPressed()
{
	cout << "call ButtonResetCloudPassthroughPressed()" << endl;
}
void MainUI::ButtonApplyCloudPassthroughPressed()
{
	//cout << "call ButtonApplyCloudPassthroughPressed()" << endl;
	dataprocess->StoreLastedOperationCloud(dataprocess->GetCurrentDisplayPointCloud());
	

	dataprocess->ApplyPassthroughFilter(
		dataprocess->GetCurrentDisplayPointCloud(),
		ui->edit_passthrough_xmin->text().toDouble(),
		ui->edit_passthrough_xmax->text().toDouble(),
		ui->edit_passthrough_ymin->text().toDouble(),
		ui->edit_passthrough_ymax->text().toDouble(),
		ui->edit_passthrough_zmin->text().toDouble(),
		ui->edit_passthrough_zmax->text().toDouble());

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetCurrentDisplayPointCloud());
	

}



void MainUI::ButtonSetCloudCenterPressed()
{
	cout << "call ButtonSetCloudCenterPressed()" << endl;
}
void MainUI::ButtonSetCloudCornerPressed()
{
	cout << "call ButtonSetCloudCornerPressed()" << endl;
	if (last_select_item_index == -1)
	{
		QMessageBox::information(0, QString("ButtonSetCloudCornerPressed"),
			QString("No Item selected"), QMessageBox::Ok);
		return;
	}

	PointCloudXYZRGB::Ptr pointcloud = dataprocess->items[last_select_item_index]->object_pointcloud;
	dataprocess->items[last_select_item_index]->transform->CalculateMinMaxPoint(pointcloud);

	double item_width = dataprocess->items[last_select_item_index]->transform->max3d_point.x - dataprocess->items[last_select_item_index]->transform->min3d_point.x;
	double item_height = dataprocess->items[last_select_item_index]->transform->max3d_point.y - dataprocess->items[last_select_item_index]->transform->min3d_point.y;
	double item_depth = dataprocess->items[last_select_item_index]->transform->max3d_point.z - dataprocess->items[last_select_item_index]->transform->min3d_point.z;

	QTreeWidgetItem* item = ui->treeWidget->topLevelItem(last_select_item_index);
	item->setText(1, QString::number(item_width));
	item->setText(2, QString::number(item_height));
	item->setText(3, QString::number(item_depth));

	PointTypeXYZRGB move_from_point = dataprocess->items[last_select_item_index]->transform->min3d_point;
	PointTypeXYZRGB move_to_point;
	move_to_point.x = 0;
	move_to_point.y = 0;
	move_to_point.z = 0;

	dataprocess->MovePointCloudFromTo(pointcloud, move_from_point, move_to_point);

	viewerembeded->UpdateCloudViewer(pointcloud);

}

void MainUI::ButtonSetCloudAlignCornerPressed()
{
	cout << "call ButtonSetCloudAlignCornerPressed()" << endl;
	if (last_select_item_index == -1)
	{
		QMessageBox::information(0, QString("ButtonSetCloudAlignCornerPressed"),
			QString("No Item selected"), QMessageBox::Ok);
		return;
	}
	cout << endl;
	cout << "major_vector " << dataprocess->items[last_select_item_index]->transform->major_vector << endl;
	cout << "minor_vector " << dataprocess->items[last_select_item_index]->transform->minor_vector << endl;
	cout << "middle_vector " << dataprocess->items[last_select_item_index]->transform->middle_vector << endl;
	
	PointCloudXYZRGB::Ptr pointcloud = dataprocess->items[last_select_item_index]->object_pointcloud;

	Eigen::Matrix<float, 1, 3>  floor_plane_normal_vector, target_plane_normal_vector;
	floor_plane_normal_vector[0] = dataprocess->items[last_select_item_index]->transform->major_vector(0);
	floor_plane_normal_vector[1] = dataprocess->items[last_select_item_index]->transform->major_vector(1);
	floor_plane_normal_vector[2] = dataprocess->items[last_select_item_index]->transform->major_vector(2);


	if (floor_plane_normal_vector[0]>0.9)
	{
		target_plane_normal_vector[0] = 1.0;
		target_plane_normal_vector[1] = 0.0;
		target_plane_normal_vector[2] = 0.0;
	}
	else if (floor_plane_normal_vector[0]<-0.9)
	{
		target_plane_normal_vector[0] = -1.0;
		target_plane_normal_vector[1] = 0.0;
		target_plane_normal_vector[2] = 0.0;
	}
	else if (floor_plane_normal_vector[2]>0.9)
	{
		target_plane_normal_vector[0] = 0.0;
		target_plane_normal_vector[1] = 0.0;
		target_plane_normal_vector[2] = 1.0;
	}
	else if (floor_plane_normal_vector[2]<-0.9)
	{
		target_plane_normal_vector[0] = 0.0;
		target_plane_normal_vector[1] = 0.0;
		target_plane_normal_vector[2] = -1.0;
	}
	else
	{
		cout << endl << "ERROR ROTATE!!!" << endl;

	}

	dataprocess->RotatePointCloudAtAxis(pointcloud,
		floor_plane_normal_vector, target_plane_normal_vector);


	viewerembeded->UpdateCloudViewer(pointcloud);
}

void MainUI::ButtonCalculateCloudTransformPressed()
{
	cout << "call ButtonCalculateCloudTransformPressed()" << endl;
	if (last_select_item_index == -1)
	{
		QMessageBox::information(0, QString("ButtonCalculateCloudTransformPressed"),
			QString("No Item selected"), QMessageBox::Ok);
		return;
	}
	PointCloudXYZRGB::Ptr pointcloud = dataprocess->items[last_select_item_index]->object_pointcloud;
	dataprocess->items[last_select_item_index]->transform->CalculateTransformation(pointcloud, 0.005);

}

//tab:segmentation
void MainUI::ButtonApplyVoxelGridPressed()
{
	cout << "call ButtonApplyVoxelGridPressed()" << endl;
	
	
	PointCloudXYZRGB::Ptr pointcloud = dataprocess->GetCurrentDisplayPointCloud();
	double voxelsize = ui->in_voxel_leafsize->text().toDouble();
	dataprocess->StoreLastedOperationCloud(pointcloud);

	dataprocess->voxelfilter->FilterVoxelSize(pointcloud, voxelsize);

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetCurrentDisplayPointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetCurrentDisplayPointCloud());
	
	cout << "voxel= " << voxelsize << ", GetCurrentDisplayPointCloudSize= " << dataprocess->GetCurrentDisplayPointCloudSize() << endl;

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

	WritePlaneParamToUI();

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetAppliedRedPlanePointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetAppliedRedPlanePointCloud());

}
void MainUI::WritePlaneParamToUI()
{
	//plane coefficients
	ui->in_planecoef_1->setText(
		QString::number(dataprocess->planeseg->transformextract->plane_coefficients_matrix(0)));
	ui->in_planecoef_2->setText(
		QString::number(dataprocess->planeseg->transformextract->plane_coefficients_matrix(1)));
	ui->in_planecoef_3->setText(
		QString::number(dataprocess->planeseg->transformextract->plane_coefficients_matrix(2)));

	//min_point_OBB
	ui->in_minobb_1->setText(
		QString::number(dataprocess->planeseg->transformextract->min_point_OBB.x));
	ui->in_minobb_2->setText(
		QString::number(dataprocess->planeseg->transformextract->min_point_OBB.y));
	ui->in_minobb_3->setText(
		QString::number(dataprocess->planeseg->transformextract->min_point_OBB.z));
	//max_point_OBB
	ui->in_maxobb_1->setText(
		QString::number(dataprocess->planeseg->transformextract->max_point_OBB.x));
	ui->in_maxobb_2->setText(
		QString::number(dataprocess->planeseg->transformextract->max_point_OBB.y));
	ui->in_maxobb_3->setText(
		QString::number(dataprocess->planeseg->transformextract->max_point_OBB.z));
	//position_OBB
	ui->in_posobb_1->setText(
		QString::number(dataprocess->planeseg->transformextract->position_OBB.x));
	ui->in_posobb_2->setText(
		QString::number(dataprocess->planeseg->transformextract->position_OBB.y));
	ui->in_posobb_3->setText(
		QString::number(dataprocess->planeseg->transformextract->position_OBB.z));

	//pca vector (principal component analysis)
	ui->in_majorvector_1->setText(
		QString::number(dataprocess->planeseg->transformextract->major_vector(0)));
	ui->in_majorvector_2->setText(
		QString::number(dataprocess->planeseg->transformextract->major_vector(1)));
	ui->in_majorvector_3->setText(
		QString::number(dataprocess->planeseg->transformextract->major_vector(2)));

	ui->in_middlevector_1->setText(
		QString::number(dataprocess->planeseg->transformextract->middle_vector(0)));
	ui->in_middlevector_2->setText(
		QString::number(dataprocess->planeseg->transformextract->middle_vector(1)));
	ui->in_middlevector_3->setText(
		QString::number(dataprocess->planeseg->transformextract->middle_vector(2)));
	
	ui->in_minorvector_1->setText(
		QString::number(dataprocess->planeseg->transformextract->minor_vector(0)));
	ui->in_minorvector_2->setText(
		QString::number(dataprocess->planeseg->transformextract->minor_vector(1)));
	ui->in_minorvector_3->setText(
		QString::number(dataprocess->planeseg->transformextract->minor_vector(2)));

	//mass center
	ui->in_masscenter_1->setText(
		QString::number(dataprocess->planeseg->transformextract->mass_center(0)));
	ui->in_masscenter_2->setText(
		QString::number(dataprocess->planeseg->transformextract->mass_center(1)));
	ui->in_masscenter_3->setText(
		QString::number(dataprocess->planeseg->transformextract->mass_center(2)));

	//min max 3d
	ui->in_min3d_1->setText(
		QString::number(dataprocess->planeseg->transformextract->min3d_point.x));
	ui->in_min3d_2->setText(
		QString::number(dataprocess->planeseg->transformextract->min3d_point.y));
	ui->in_min3d_3->setText(
		QString::number(dataprocess->planeseg->transformextract->min3d_point.z));
	
	ui->in_max3d_1->setText(
		QString::number(dataprocess->planeseg->transformextract->max3d_point.x));
	ui->in_max3d_2->setText(
		QString::number(dataprocess->planeseg->transformextract->max3d_point.y));
	ui->in_max3d_3->setText(
		QString::number(dataprocess->planeseg->transformextract->max3d_point.z));

	//rotatoin matrix
	ui->in_rotationmatrix_1->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(0)));
	ui->in_rotationmatrix_2->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(1)));
	ui->in_rotationmatrix_3->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(2)));

	ui->in_rotationmatrix_4->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(3)));
	ui->in_rotationmatrix_5->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(4)));
	ui->in_rotationmatrix_6->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(5)));

	ui->in_rotationmatrix_7->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(6)));
	ui->in_rotationmatrix_8->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(7)));
	ui->in_rotationmatrix_9->setText(
		QString::number(dataprocess->planeseg->transformextract->rotational_matrix_OBB(8)));
}

void MainUI::ButtonCalculatePlaneTransformPressed()
{
	cout << "call ButtonCalculatePlaneTransformPressed()" << endl;

	double planethreshold = ui->in_plane_threshold->text().toDouble();
	dataprocess->planeseg->CalculatePlaneTransformation(planethreshold, dataprocess->GetOnlyPlanePointCloud());

	dataprocess->planeseg->RemovePlaneOutside(dataprocess->GetCurrentDisplayPointCloud());
	cout << "After RemovePlaneOutside PointCloudSize= " << dataprocess->GetCurrentDisplayPointCloudSize() << endl;


	WritePlaneParamToUI();

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetRemovedPlaneOutsidePointCloud());
	dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetRemovedPlaneOutsidePointCloud());

	viewerwindow->AddBoundingBoxWindowCloudViewer(
		dataprocess->planeseg->transformextract->position_OBB,
		dataprocess->planeseg->transformextract->min_point_OBB,
		dataprocess->planeseg->transformextract->max_point_OBB,
		dataprocess->planeseg->transformextract->rotational_matrix_OBB,
		"planeseg OBB"
		);

	string id_name = " plane";

	//min_point_OBB
	viewerwindow->AddSphereWindowCloudViewer(
	dataprocess->planeseg->transformextract->min_point_OBB, 0.01,
		0, 1, 0.0, " minobb" + id_name);
	viewerwindow->AddTextWindowCloudViewer(
		dataprocess->planeseg->transformextract->min_point_OBB, 0.01,
		1.0, 1.0, 1.0, "minobb" + id_name, "text minobb" + id_name);

	//max_point_OBB
	viewerwindow->AddSphereWindowCloudViewer(
		dataprocess->planeseg->transformextract->max_point_OBB, 0.01,
		0, 1, 0.0, " maxobb" + id_name);
	viewerwindow->AddTextWindowCloudViewer(
		dataprocess->planeseg->transformextract->max_point_OBB, 0.01,
		1.0, 1.0, 1.0, "maxobb" + id_name, " text maxobb" + id_name);

	//min_point_3d
	viewerwindow->AddSphereWindowCloudViewer(
		dataprocess->planeseg->transformextract->min3d_point, 0.01,
		0, 0, 1.0, " min3d" + id_name);
	viewerwindow->AddTextWindowCloudViewer(
		dataprocess->planeseg->transformextract->min3d_point, 0.01,
		1.0, 1.0, 1.0, "min3d" + id_name, "text min3d" + id_name);

	//max_point_3d
	viewerwindow->AddSphereWindowCloudViewer(
		dataprocess->planeseg->transformextract->max3d_point, 0.01,
		0, 0, 1.0, " max3d" + id_name);
	viewerwindow->AddTextWindowCloudViewer(
		dataprocess->planeseg->transformextract->max3d_point, 0.01,
		1.0, 1.0, 1.0, "max3d" + id_name, " text max3d" + id_name);


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

	int pointcloudsize = static_cast<int>(dataprocess->GetRemovedPlanePointCloud()->size());

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
	viewerwindow->ToggleAxisONWindowCloudViewer();

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

	int pointcloudsize = static_cast<int>(dataprocess->GetCurrentDisplayPointCloud()->size());

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

	viewerwindow->DrawPlanarAtOrigin(0.5, 1.0, 1.0, 1.0, "planeorigin");


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

	ButtonClearAllItemPressed();

	dataprocess->SeparateContainerAndItems(dataprocess->clusterextract->GetExtractCluster());
	
	dataprocess->CalculateContainerTransformation();


	

	ShowMinMaxCenterPoint(dataprocess->container, "container");
	ui->in_bin_x_dim->setText(QString::number(dataprocess->container->x_length_mm));
	ui->in_bin_y_dim->setText(QString::number(dataprocess->container->y_length_mm));
	ui->in_bin_z_dim->setText(QString::number(dataprocess->container->z_length_mm));
	ui->in_pos_container_1->setText(QString::number(dataprocess->container->transform->position_OBB.x));
	ui->in_pos_container_2->setText(QString::number(dataprocess->container->transform->position_OBB.y));
	ui->in_pos_container_3->setText(QString::number(dataprocess->container->transform->position_OBB.z));

	dataprocess->CalculateItemsTransformation();
	for (int i = 0; i < dataprocess->items.size();i++)
	{
		ShowMinMaxCenterPoint(dataprocess->items[i], "item_" + to_string(i+1));

		QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);

		item->setText(0, QString::number(i+1));
		item->setText(1, QString::number(dataprocess->items[i]->x_length_mm));
		item->setText(2, QString::number(dataprocess->items[i]->y_length_mm));
		item->setText(3, QString::number(dataprocess->items[i]->z_length_mm));
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
void MainUI::ButtonShowClusterVectorPressed()
{

}


//cluster list
void MainUI::PressedTreeItem(QTreeWidgetItem *current_select_item)
{
	cout << "call PressedTreeItem()" << endl;

	if (last_select_item_index != -1)// if has already hilight previous one
	{
		//remove hilight last selected item
		QTreeWidgetItem* last_selected_item = ui->treeWidget->topLevelItem(last_select_item_index);
		//last_selected_item->setBackgroundColor(0, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(1, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(2, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(3, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(4, QColor(255, 255, 255));
	}


	
	//hilight item clicked
	//item->setBackgroundColor(0, QColor(200, 200, 200));
	current_select_item->setBackgroundColor(1, QColor(200, 200, 200));
	current_select_item->setBackgroundColor(2, QColor(200, 200, 200));
	current_select_item->setBackgroundColor(3, QColor(200, 200, 200));
	current_select_item->setBackgroundColor(4, QColor(200, 200, 200));

	last_select_item_index = ui->treeWidget->currentIndex().row();



	cout << "select " << last_select_item_index << endl;
	//show seleted cloud
	//program->ShowSelectedListedCloudIndex(last_select_item_index);


	PointCloudXYZRGB::Ptr pointcloud = dataprocess->items[last_select_item_index]->object_pointcloud;
	viewerembeded->UpdateCloudViewer(pointcloud);
};
void MainUI::ButtonLoadPointCloudToListPressed()
{
	cout << "call ButtonLoadPointCloudToListPressed()" << endl;

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Load pcd"), POINTCLOUD_DIR, tr("point cloud (*.pcd)"));
	if (filename.trimmed().isEmpty()) return;



	PointCloudXYZRGB::Ptr cloud = dataprocess->LoadPcdFileToPointCloudVariable(filename.toStdString());
	//copy cloud to dataprocess->items[i]
	dataprocess->AddLoadPointCloudToItems(cloud);

	int pointcloud_number=ui->treeWidget->topLevelItemCount()+1;
	
	QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget); // at this  point  topLevelItemCount() is increase
	item->setText(0, QString::number(pointcloud_number));
	item->setText(4, QString::number(cloud->size()));
	item->setText(11, filename);
	
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
void MainUI::ButtonSavePointCloudFromListPressed()
{
	cout << "call ButtonSavePointCloudFromListPressed()" << endl;

	if (last_select_item_index == -1)
	{
		QMessageBox::information(0, QString("ButtonSavePointCloudFromListPressed"), 
			QString("No Item selected"), QMessageBox::Ok);
		return;
	}

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save pcd"), POINTCLOUD_DIR, tr("point cloud (*.pcd)"));
	if (filename.trimmed().isEmpty()) return;

	dataprocess->SavePointCloud(filename.toStdString(),
		dataprocess->items[last_select_item_index]->object_pointcloud);

	QTreeWidgetItem* last_selected_item = ui->treeWidget->topLevelItem(last_select_item_index);
	last_selected_item->setText(11, filename);


}


void MainUI::ButtonAlignAllItemAxisPressed()
{
	cout << "call ButtonAlignAllItemAxisPressed()" << endl;

	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		last_select_item_index = i;
		ButtonSetCloudCornerPressed();
	}
	dataprocess->isSetAlignCorner = true;

}
void MainUI::ButtonSaveAllItemPcdPressed()
{
	cout << "call ButtonSaveAllItemPcdPressed()" << endl;

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save pcd"), POINTCLOUD_DIR);
	if (filename.trimmed().isEmpty()) return;

	

	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		string eachfilename = filename.toStdString() + to_string(i + 1)+".pcd";
		dataprocess->SavePointCloud(eachfilename,
			dataprocess->items[0]->object_pointcloud);

		cout << eachfilename << endl;

		QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
		item->setText(11, QString::fromStdString(eachfilename));
	}


}

void MainUI::ButtonLoadAllItemPressed()
{
	cout << "call ButtonLoadAllItemPressed()" << endl;

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Load container & items list file"), POINTCLOUD_DIR, tr("text file (*.txt)"));
	if (filename.trimmed().isEmpty()) return;

	int total_items;
	int container_w, container_h, container_d;
	int container_x, container_y, container_z;
	double container_orient_x, container_orient_y, container_orient_z;
	vector<string> array_items_filename;
	int *items_w, *items_h, *items_d;
	int *items_x, *items_y, *items_z;
	double *items_orient_x, *items_orient_y, *items_orient_z;
	
	total_items=dataprocess->pointcloudbpptext->ReadPointCloudListForBPP(filename.toStdString());

	container_w = dataprocess->pointcloudbpptext->bin_width;
	container_h = dataprocess->pointcloudbpptext->bin_height;
	container_d = dataprocess->pointcloudbpptext->bin_depth;

	array_items_filename = dataprocess->pointcloudbpptext->array_pcd_filename;

	items_w = dataprocess->pointcloudbpptext->item_w;
	items_h = dataprocess->pointcloudbpptext->item_h;
	items_d = dataprocess->pointcloudbpptext->item_d;

	//call read pcd file
	//dataprocess->LoadTxtListofItems(filename.toStdString());


	//add load txt item to ui

	for (int i = 0; i < total_items; i++)
	{
		//call read pcd file
		PointCloudXYZRGB::Ptr cloud=dataprocess->LoadPcdFileToPointCloudVariable(array_items_filename[i]);

		//copy cloud to dataprocess->items[i]
		dataprocess->AddLoadPointCloudToItems(cloud);
	
		int pointcloud_number = ui->treeWidget->topLevelItemCount() + 1;
		QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget); // when new item, topLevelItemCount() is increase
		item->setText(0, QString::number(i + 1));
		item->setText(1, QString::number(items_w[i]));
		item->setText(2, QString::number(items_d[i]));
		item->setText(3, QString::number(items_h[i]));
		item->setText(4, QString::number(cloud->size()));
		item->setText(11, QString::fromStdString(array_items_filename[i]));

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
void MainUI::ButtonSaveAllItemPressed()
{
	cout << "call ButtonSaveAllItemPressed()" << endl;



	vector<string> array_pcd_filename;
	int total_boxes = ui->treeWidget->topLevelItemCount();
	int *boxes_width = new int[total_boxes];
	int *boxes_height = new int[total_boxes];
	int *boxes_depth = new int[total_boxes];

	for (int i = 0; i < total_boxes; i++)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		if (item->text(11) == "")
		{
			//cout << i << " no filename" << endl;
			QMessageBox::information(0, QString("ButtonSaveAllItemPressed"), QString("Each Pcd File has not been saved"), QMessageBox::Ok);
			return;
		}
		else
		{
			array_pcd_filename.push_back(item->text(11).toStdString());
		}

		if (item->text(1) == "")
		{
			QMessageBox::information(0, QString("ButtonSaveAllItemPressed"), QString("Pointcloud has no WHD Data"), QMessageBox::Ok);
			return;
		}
		else
		{
			boxes_width[i] = item->text(1).toInt();
			boxes_height[i] = item->text(3).toInt();
			boxes_depth[i] = item->text(2).toInt();
		}
	}

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save txt"), POINTCLOUD_DIR, tr("list of pcd (*.txt)"));

	if (filename.trimmed().isEmpty()) return;


	dataprocess->pointcloudbpptext->WritePointCloudListForBPP(
		filename.toStdString(), total_boxes,
		ui->in_bin_x_dim->text().toDouble(),
		ui->in_bin_y_dim->text().toDouble(),
		ui->in_bin_z_dim->text().toDouble(),
		array_pcd_filename,
		boxes_width, boxes_height, boxes_depth
		);


}
void MainUI::ButtonRemoveItemPressed()
{
	cout << "call ButtonRemoveItemPressed()" << endl;

	QTreeWidgetItem* last_selected_item = ui->treeWidget->topLevelItem(last_select_item_index);

	delete last_selected_item;

	int total_boxes = ui->treeWidget->topLevelItemCount();

	//rerun index number
	for (int i = 0; i < total_boxes; ++i)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		item->setText(0, QString::number(i + 1));

	}

}
void MainUI::ButtonClearAllItemPressed()
{
	cout << "call ButtonClearAllItemPressed()" << endl;
	
	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(0);
		delete item;
	}

	viewerembeded->ClearPointCloudEmbededCloudViewer();
	last_select_item_index = -1;

}
void MainUI::ButtonCalculateBinPackingPressed()
{
	cout << "call ButtonCalculateBinPackingPressed()" << endl;

	int total_boxes = ui->treeWidget->topLevelItemCount();

	if (total_boxes != dataprocess->items.size())
	{
		cout << "total_boxes != dataprocess->items.size()  return" << endl;
		return;
	}

	if (dataprocess->isSetAlignCorner == false)
	{
		ButtonAlignAllItemAxisPressed();
	}


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
		boxes_height[i] = item->text(3).toInt();
		boxes_depth[i] = item->text(2).toInt();
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


	dataprocess->CalculateBinpack(
		total_boxes,
		ui->in_bin_x_dim->text().toDouble(),
		ui->in_bin_y_dim->text().toDouble(),
		ui->in_bin_z_dim->text().toDouble(),
		boxes_w, boxes_h, boxes_d,
		boxes_x_pos, boxes_y_pos, boxes_z_pos,
		boxes_x_orient, boxes_y_orient, boxes_z_orient,
		boxes_bin_num, boxes_item_num);

	// check if use bin >1
	int item_fit = 0;
	for (int i = 0; i < total_boxes; i++)
	{

		if (boxes_bin_num[i] == 1)
		{
			item_fit++;

			ObjectTransformationData *itm = dataprocess->items[i];

			//do data in ui and data in items[i] is equal??
			itm->target_position.x = boxes_x_pos[i] * 0.001;
			itm->target_position.y = boxes_y_pos[i] * 0.001;
			itm->target_position.z = boxes_z_pos[i] * 0.001;

			itm->target_orientation.x = boxes_x_orient[i];
			itm->target_orientation.y = boxes_y_orient[i];
			itm->target_orientation.z = boxes_z_orient[i];

			if (itm->x_length_mm == boxes_x_orient[i] &&
				itm->y_length_mm == boxes_y_orient[i] &&
				itm->z_length_mm == boxes_z_orient[i])
			{
				itm->rotation_case = 1;
			}
			else if (itm->x_length_mm == boxes_z_orient[i] &&
				itm->y_length_mm == boxes_y_orient[i] &&
				itm->z_length_mm == boxes_x_orient[i])
			{
				itm->rotation_case = 2;
			}
			else if (itm->x_length_mm == boxes_x_orient[i] &&
				itm->y_length_mm == boxes_z_orient[i] &&
				itm->z_length_mm == boxes_y_orient[i])
			{
				itm->rotation_case = 3;
			}
			else if (itm->x_length_mm == boxes_y_orient[i] &&
				itm->y_length_mm == boxes_x_orient[i] &&
				itm->z_length_mm == boxes_z_orient[i])
			{
				itm->rotation_case = 4;
			}
			else if (itm->x_length_mm == boxes_z_orient[i] &&
				itm->y_length_mm == boxes_x_orient[i] &&
				itm->z_length_mm == boxes_y_orient[i])
			{
				itm->rotation_case = 5;
			}
			else if (itm->x_length_mm == boxes_y_orient[i] &&
				itm->y_length_mm == boxes_z_orient[i] &&
				itm->z_length_mm == boxes_x_orient[i])
			{
				itm->rotation_case = 6;
			}
		}

	}
	cout << endl;
	cout << "packed " << item_fit << "/" << total_boxes << endl;

	//binpack->SortBoxesOrder();



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

	/*viewerwindow->ShowBinpackingIndication(
		dataprocess->container->transform->min3d_point,
		total_boxes,
		ui->in_bin_w->text().toDouble(),
		ui->in_bin_d->text().toDouble(),
		ui->in_bin_h->text().toDouble(),
		boxes_x_orient, boxes_y_orient, boxes_z_orient,
		boxes_x_pos, boxes_y_pos, boxes_z_pos);
*/
}






void MainUI::ButtonShowPackingTargetPressed()
{
	cout << "call ButtonShowPackingPressed()" << endl;
	
	//hilight circle at each item (input_position)
	for (int i = 0; i < dataprocess->items.size(); i++)
	{
		//cout << "i=" << i << " x=" << dataprocess->items[i]->x_length << " z=" << dataprocess->items[i]->z_length << endl;
		float radius;
		if (dataprocess->items[i]->x_length > dataprocess->items[i]->z_length)
		{
			radius = dataprocess->items[i]->x_length*0.5;
		}
		else
		{
			radius = dataprocess->items[i]->z_length*0.5;
		}

		viewerwindow->AddCircleWindowCloudViewer(dataprocess->items[i]->transform->position_OBB,
			radius, 1.0, 0.0, 0.0, "circle " + i);

		

	}

	//print packing output at console
	for (int i = 0; i < dataprocess->items.size(); i++)
	{
		/*cout << "i = " << i << endl
			<< " dim = " << dataprocess->items[i]->x_length_mm << 
			" " << dataprocess->items[i]->y_length_mm << 
			" " << dataprocess->items[i]->z_length_mm << endl
			<< " pos =" << dataprocess->items[i]->target_position << endl
			<< " orient = " << dataprocess->items[i]->target_orientation << endl
			<< endl;*/
		cout << "i = " << i << endl
			<< " pos =" << dataprocess->items[i]->target_position << endl
			<< " rotation_case = " << dataprocess->items[i]->rotation_case << endl
			<< endl;
	}

	viewerwindow->ShowBinPackingTarget(
	dataprocess->container->transform->min3d_point,
	dataprocess->items);
	
	
}
void MainUI::ButtonShowPackingIndicatePressed()
{
	cout << "call ButtonShowPackingIndicatePressed()" << endl;

}
void MainUI::ButtonShowPackingAnimationPressed()
{
	cout << "call ButtonShowPackingAnimationPressed()" << endl;



}

void MainUI::ButtonShowZeroPackingPressed()
{
	cout << "call ButtonShowZeroPackingPressed()" << endl;

}
void MainUI::ButtonShowPrevPackingPressed()
{
	cout << "call ButtonShowPrevPackingPressed()" << endl;

}
void MainUI::ButtonShowNextPackingPressed()
{
	cout << "call ButtonShowNextPackingPressed()" << endl;

}

void MainUI::ButtonSaveBinPackingInfoPressed()
{

}
void MainUI::ButtonLoadBinPackingInfoPressed()
{

}


