#include "MainUI.h"

#define POINTCLOUD_DIR "../pcd_files"
#define COLUMN_FILENAME 13


MainUI::MainUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainUI)
{
	cout << "MainUI::MainUI(QWidget *parent)" << endl;

    ui->setupUi(this);

	timerId_kinect = 0;
	isLoadPlaneParameter = false;

	last_select_item_index = -1;
	last_select_sorting_index = -1;
	
	current_display_packing_number = 0;
	ui->bt_order_one->setText(QString::number(current_display_packing_number));


	viewerembeded = new ViewerEmbeded(ui->widget);

	//ui->in_point_size_visual->text().toInt()


	//top menu
	///menu:viewer embedded
	connect(ui->action_connectkinect, SIGNAL(triggered()), this, SLOT(ButtonConnectPressed()));
	connect(ui->action_disconnectkinect, SIGNAL(triggered()), this, SLOT(ButtonDisconnectPressed()));
	connect(ui->action_loadpointcloud, SIGNAL(triggered()), this, SLOT(ButtonLoadPointCloudToViewerPressed()));
	connect(ui->action_savepointcloud, SIGNAL(triggered()), this, SLOT(ButtonSavePointCloudFromViewerPressed()));

	//menu:parameter
	connect(ui->action_load_transform, SIGNAL(triggered()), this, SLOT(ButtonLoadTransformParamPressed()));
	connect(ui->action_save_transform, SIGNAL(triggered()), this, SLOT(ButtonSaveTransformParamPressed()));
	connect(ui->action_load_passthrough, SIGNAL(triggered()), this, SLOT(ButtonLoadPassthroughParamPressed()));
	connect(ui->action_save_passthrough, SIGNAL(triggered()), this, SLOT(ButtonSavePassthroughParamPressed()));
	connect(ui->action_load_camera, SIGNAL(triggered()), this, SLOT(ButtonLoadCameraParamPressed()));
	connect(ui->action_save_camera, SIGNAL(triggered()), this, SLOT(ButtonSaveCameraParamPressed()));
	connect(ui->action_load_plane, SIGNAL(triggered()), this, SLOT(ButtonLoadPlaneParamPressed()));
	connect(ui->action_save_plane, SIGNAL(triggered()), this, SLOT(ButtonSavePlaneParamPressed()));
	connect(ui->action_load_ui_items, SIGNAL(triggered()), this, SLOT(ButtonLoadAllItemsTextToUIPressed()));
	connect(ui->action_save_ui_items, SIGNAL(triggered()), this, SLOT(ButtonSaveAllItemsUIToTextPressed()));
	connect(ui->action_load_bpp_info, SIGNAL(triggered()), this, SLOT(ButtonLoadBinPackingInfoPressed()));
	connect(ui->action_save_bpp_info, SIGNAL(triggered()), this, SLOT(ButtonSaveBinPackingInfoPressed()));

	//items above tab
	connect(ui->radio_axis_on, SIGNAL(clicked()), this, SLOT(RadioButtonAxisONSelected()));
	connect(ui->radio_axis_off, SIGNAL(clicked()), this, SLOT(RadioButtonAxisOFFSelected()));
	
	connect(ui->bt_reloadcloud_to_viewer, SIGNAL(clicked()), this, SLOT(ButtonShowLoadedPointCloudPressed()));
	connect(ui->bt_clear_viewer_pointcloud, SIGNAL(clicked()), this, SLOT(ButtonClearViewerPointCloudPressed()));
	connect(ui->bt_clear_viewer_shape, SIGNAL(clicked()), this, SLOT(ButtonClearViewerShapePressed()));
	connect(ui->bt_undo_last_operation, SIGNAL(clicked()), this, SLOT(ButtonUndoLastedPointCloudPressed()));
	
	//tab:viewer parameter
	connect(ui->bt_apply_param_to_cam_window, SIGNAL(clicked()), this, SLOT(ButtonApplyParamtoCameraWindowPressed()));
	connect(ui->bt_apply_param_to_cam_embeded, SIGNAL(clicked()), this, SLOT(ButtonApplyParamtoCameraEmbededPressed()));
	connect(ui->bt_reset_viewerparam, SIGNAL(clicked()), this, SLOT(ButtonResetCamParamPressed()));
	connect(ui->bt_apply_camrot, SIGNAL(clicked()), this, SLOT(ButtonApplyCamRotationPressed()));
	connect(ui->bt_apply_campos, SIGNAL(clicked()), this, SLOT(ButtonApplyCamTranslationPressed()));
	connect(ui->bt_get_cam_param, SIGNAL(clicked()), this, SLOT(ButtonGetRegisterCameraCallbackPressed()));
	connect(ui->bt_load_camera_txt, SIGNAL(clicked()), this, SLOT(ButtonLoadCameraParamPressed()));
	connect(ui->bt_save_camera_txt, SIGNAL(clicked()), this, SLOT(ButtonSaveCameraParamPressed()));

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
	connect(ui->bt_set_cloud_align_axis, SIGNAL(clicked()), this, SLOT(ButtonSetCloudAlignAxisPressed()));
	connect(ui->bt_calculate_cloud_transform, SIGNAL(clicked()), this, SLOT(ButtonCalculateCloudTransformPressed()));
	connect(ui->bt_fill_empty, SIGNAL(clicked()), this, SLOT(ButtonFillEmptyPressed()));
	connect(ui->bt_fill_invert, SIGNAL(clicked()), this, SLOT(ButtonFillInvertPressed()));


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
	connect(ui->bt_save_all_items_to_pcd, SIGNAL(clicked()), this, SLOT(ButtonSaveAllItemPointcloudToPcdPressed()));

	connect(ui->bt_fill_empty_all, SIGNAL(clicked()), this, SLOT(ButtonFillEmptyAllPressed()));
	connect(ui->bt_fill_invert_all, SIGNAL(clicked()), this, SLOT(ButtonFillInvertAllPressed()));

	connect(ui->bt_all_load, SIGNAL(clicked()), this, SLOT(ButtonLoadAllItemsTextToUIPressed()));
	connect(ui->bt_all_save, SIGNAL(clicked()), this, SLOT(ButtonSaveAllItemsUIToTextPressed()));

	connect(ui->bt_item_remove, SIGNAL(clicked()), this, SLOT(ButtonRemoveItemPressed()));
	connect(ui->bt_item_clearall, SIGNAL(clicked()), this, SLOT(ButtonClearAllItemPressed()));

	//packing

	connect(ui->bt_binpacking, SIGNAL(clicked()), this, SLOT(ButtonCalculateBinPackingPressed()));

	connect(ui->bt_show_packing_target, SIGNAL(clicked()), this, SLOT(ButtonShowPackingTargetPressed()));
	connect(ui->bt_show_packing_indicate, SIGNAL(clicked()), this, SLOT(ButtonShowPackingIndicatePressed()));
	connect(ui->bt_show_packing_animation, SIGNAL(clicked()), this, SLOT(ButtonShowPackingAnimationPressed()));

	connect(ui->bt_show_input_rectangle, SIGNAL(clicked()), this, SLOT(ButtonShowProjectionInputPositionPressed()));
	connect(ui->bt_test_project_rectangle, SIGNAL(clicked()), this, SLOT(ButtonShowTestRectanglePositionPressed()));
	connect(ui->bt_test_project_cube, SIGNAL(clicked()), this, SLOT(ButtonShowTestCubePressed()));

	connect(ui->bt_set_minpoint_y_zero, SIGNAL(clicked()), this, SLOT(ButtonSetContainerItemsYzeroPressed()));
	connect(ui->bt_update_dataprocess_from_ui, SIGNAL(clicked()), this, SLOT(ButtonUpdateContainerItemsToDataprocessPressed()));


	//sorting
	connect(ui->treeWidgetSorting, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(PressedTreeSorting(QTreeWidgetItem *)));

	connect(ui->bt_order_moveup, SIGNAL(clicked()), this, SLOT(ButtonMoveUpPackingOrderPressed()));
	connect(ui->bt_order_movedown, SIGNAL(clicked()), this, SLOT(ButtonMoveDownPackingOrderPressed()));
	connect(ui->bt_update_packing_order, SIGNAL(clicked()), this, SLOT(ButtonUpdatePackingOrderPressed()));

	connect(ui->bt_order_one, SIGNAL(clicked()), this, SLOT(ButtonShowZeroPackingPressed()));
	connect(ui->bt_order_previous, SIGNAL(clicked()), this, SLOT(ButtonShowPrevPackingPressed()));
	connect(ui->bt_order_next, SIGNAL(clicked()), this, SLOT(ButtonShowNextPackingPressed()));

	connect(ui->bt_save_packing_info, SIGNAL(clicked()), this, SLOT(ButtonSaveBinPackingInfoPressed()));
	connect(ui->bt_load_packing_info, SIGNAL(clicked()), this, SLOT(ButtonLoadBinPackingInfoPressed()));
	

	//test
	connect(ui->bt_test, SIGNAL(clicked()), this, SLOT(ButtonTestProgrammePressed()));
	connect(ui->bt_test_input1, SIGNAL(clicked()), this, SLOT(ButtonTestInput1Pressed()));
	connect(ui->bt_test_input2, SIGNAL(clicked()), this, SLOT(ButtonTestInput2Pressed()));
	connect(ui->bt_test_input3, SIGNAL(clicked()), this, SLOT(ButtonTestInput3Pressed()));
	connect(ui->bt_test_input4, SIGNAL(clicked()), this, SLOT(ButtonTestInput4Pressed()));
	
	//
	//
	//
	//



	//resize tree column size	

	for (int j = 0; j < 5; j++)
	{
		ui->treeWidget->header()->resizeSection(j, 45);
	}

	for (int j = 5; j < 11; j++)
	{
		ui->treeWidget->header()->resizeSection(j, 90);
	}
	ui->treeWidget->header()->resizeSection(11, 45);
	ui->treeWidget->header()->resizeSection(12, 45);
	ui->treeWidget->header()->resizeSection(COLUMN_FILENAME, 500);

	//QApplication::instance()->installEventFilter(this);


	
}

MainUI::~MainUI()
{
    //delete ui;
	//delete program;
}

void MainUI::ButtonTestProgrammePressed()
{

	viewerwindow->AddArrowObj();



}
void MainUI::ButtonTestInput1Pressed()
{

	Call_LoadCameraParam("C:/Users/nattaon2/Desktop/bpp_project_december/_camera_topview_param_lab3.txt");
	Call_LoadAllItemsTextToUI("C:/Users/nattaon2/Desktop/bpp_project_december/pcd_files/12/tt_lab_size_pos_correction.txt");
	Call_LoadBinPackingInfo("C:/Users/nattaon2/Desktop/bpp_project_december/pcd_files/12/packing12reorder.txt");
/*
	Call_LoadCameraParam("C:/Users/Nattaon/Desktop/bpp_project_december/_camera_topview_param_lab_rectangle.txt");
	Call_LoadAllItemsTextToUI("C:/Users/Nattaon/Desktop/bpp_project_december/pcd_files/12/tt_home_size_pos_correction.txt");
	Call_LoadBinPackingInfo("C:/Users/Nattaon/Desktop/bpp_project_december/pcd_files/12/packing12reorder.txt");
*/
}
void MainUI::ButtonTestInput2Pressed()
{
	PointCloudXYZRGB::Ptr pointcloud(new PointCloudXYZRGB);
	//pointcloud.reset(new PointCloudXYZRGB);

	viewerwindow->AddPointCloudPolygonMesh(pointcloud);
}
void MainUI::ButtonTestInput3Pressed()
{

}
void MainUI::ButtonTestInput4Pressed()
{

}

void MainUI::SetDataProcess(DataProcess* d) {dataprocess = d;}
void MainUI::SetViewerWindow(ViewerWindow* v) {viewerwindow = v;}

bool MainUI::eventFilter(QObject *object, QEvent *event) 
{
	if (event->type() == QEvent::KeyPress) {
		QKeyEvent* key_event = static_cast<QKeyEvent*>(event);
		//cout << "key " << key_event->key() << " object:" << object->objectName().toStdString() << endl;


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
	//cout << "MainUI event->key() " << event->key() << endl;

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
		ButtonShowPrevPackingPressed();
	}
	else if (event->key() == Qt::Key_Right)
	{
		ButtonShowNextPackingPressed();
	}

	QWidget::keyPressEvent(event);

}
void MainUI::ButtonGetRegisterCameraCallbackPressed()
{
	viewerwindow->window_view->registerMouseCallback(&MainUI::mouseEventOccurred, *this, (void*)&viewerwindow->window_view);

}
void MainUI::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void *stop_void)
{	//trigged when : mouse position is move in viewer area
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

	//check mouse click and scroll action
	if (event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		//cout << "Buttonrelease " << endl;
		SetCurrentCameraParameterToUI();

	}	
	else if (event.getButton() == pcl::visualization::MouseEvent::MouseScrollUp || event.getButton() == pcl::visualization::MouseEvent::MouseScrollDown)
	{
		//cout << "MiddleButton " << endl;
		SetCurrentCameraParameterToUI();
	}
/*	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
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
	}*/

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

	Call_LoadCameraParam(filename.toStdString());


}

void MainUI::Call_LoadCameraParam(string filename)
{


	double focal_x, focal_y, focal_z;
	double pos_x, pos_y, pos_z;
	double up_x, up_y, up_z;
	double clipping_near, clipping_far;
	double cam_angle_rad;
	double cam_angle_deg;

	vector<pcl::visualization::Camera> cam;
	viewerwindow->window_view->getCameras(cam);

	dataprocess->cameraparam->ReadCameraParameter(filename,
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
	RadioButtonAxisOFFSelected();
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
void MainUI::ButtonApplyParamtoCameraEmbededPressed()
{

}
void MainUI::ButtonApplyParamtoCameraWindowPressed()
{
	cout << "call ButtonApplyParamtoCameraWindowPressed()" << endl;

	double focal_x, focal_y, focal_z;
	double pos_x, pos_y, pos_z;
	double up_x, up_y, up_z;
	double clipping_near, clipping_far;
	double cam_angle_rad;
	double cam_angle_deg;

	focal_x = ui->in_focal_x->text().toDouble();
	focal_y = ui->in_focal_y->text().toDouble();
	focal_z = ui->in_focal_z->text().toDouble();
	pos_x = ui->in_pos_x->text().toDouble();
	pos_y = ui->in_pos_y->text().toDouble();
	pos_z = ui->in_pos_z->text().toDouble();
	up_x = ui->in_up_x->text().toDouble();
	up_y = ui->in_up_y->text().toDouble();
	up_z = ui->in_up_z->text().toDouble();
	clipping_near = ui->in_clipping_near->text().toDouble();
	clipping_far = ui->in_clipping_far->text().toDouble();
	cam_angle_rad = ui->in_cam_angle_rad->text().toDouble();
	cam_angle_deg = ui->in_cam_angle_deg->text().toDouble();

	viewerwindow->SetCameraParameter(
		focal_x, focal_y, focal_z,
		pos_x, pos_y, pos_z,
		up_x, up_y, up_z,
		clipping_near, clipping_far,
		cam_angle_rad, cam_angle_deg);


}
void MainUI::ButtonResetCamParamPressed()
{
	cout << "call ButtonResetCamParamPressed()" << endl;
	viewerwindow->window_view->initCameraParameters();
	viewerwindow->window_view->spinOnce();
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

	dataprocess->RotatePointCloudAroundZeroPoint(pointcloud,
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

void MainUI::ButtonFillEmptyPressed()
{
	//cout << "call ButtonFillEmptyPressed()" << endl;
	if (last_select_item_index == -1)
	{
		QMessageBox::information(0, QString("ButtonFillEmptyPressed"),
			QString("No Item selected"), QMessageBox::Ok);
		return;
	}

	ObjectTransformationData *item_transform_data = dataprocess->items[last_select_item_index];
	PointCloudXYZRGB::Ptr pointcloud = item_transform_data->object_pointcloud;

	dataprocess->SurfaceFillCloud(pointcloud, 0.001, item_transform_data->x_length, item_transform_data->y_length, item_transform_data->z_length);

	viewerembeded->UpdateCloudViewer(pointcloud);

}
void MainUI::ButtonFillInvertPressed()
{
	//cout << "call ButtonFillEmptyPressed()" << endl;
	if (last_select_item_index == -1)
	{
		QMessageBox::information(0, QString("ButtonFillEmptyPressed"),
			QString("No Item selected"), QMessageBox::Ok);
		return;
	}

	ObjectTransformationData *item_transform_data = dataprocess->items[last_select_item_index];
	PointCloudXYZRGB::Ptr pointcloud = item_transform_data->object_pointcloud;

	dataprocess->DuplicateInvertCloud(pointcloud, 
		item_transform_data->x_length, item_transform_data->y_length, item_transform_data->z_length);

	viewerembeded->UpdateCloudViewer(pointcloud);

}
void MainUI::ButtonFillEmptyAllPressed()
{
	//cout << "call ButtonFillInvertAllPressed()" << endl;
	int current_select_index = last_select_item_index;
	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		last_select_item_index = i;
		ButtonFillEmptyPressed();
	}
	PressedTreeItem(ui->treeWidget->topLevelItem(current_select_index));


}
void MainUI::ButtonFillInvertAllPressed()
{
	//cout << "call ButtonFillInvertAllPressed()" << endl;
	int current_select_index = last_select_item_index;
	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		last_select_item_index = i;
		ButtonFillInvertPressed();
	}
	PressedTreeItem(ui->treeWidget->topLevelItem(current_select_index));

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

	ObjectTransformationData *item_transform_data = dataprocess->items[last_select_item_index];
	PointCloudXYZRGB::Ptr pointcloud = item_transform_data->object_pointcloud;
	item_transform_data->transform->CalculateMinMaxPoint(pointcloud);

	int item_x_dim_mm = 1000 * (item_transform_data->transform->max3d_point.x - item_transform_data->transform->min3d_point.x);
	int item_y_dim_mm = 1000 * (item_transform_data->transform->max3d_point.y);// -item_transform_data->transform->min3d_point.y;
	int item_z_dim_mm = 1000 * (item_transform_data->transform->max3d_point.z - item_transform_data->transform->min3d_point.z);

	
	QTreeWidgetItem* item = ui->treeWidget->topLevelItem(last_select_item_index);
	item->setText(1, QString::number(item_x_dim_mm));
	item->setText(2, QString::number(item_y_dim_mm));
	item->setText(3, QString::number(item_z_dim_mm));

	

	PointTypeXYZRGB move_from_point = item_transform_data->transform->min3d_point;
	PointTypeXYZRGB move_to_point;
	move_to_point.x = 0;
	move_to_point.y = item_transform_data->transform->min3d_point.y; // not move y
	move_to_point.z = 0;

	dataprocess->MovePointCloudFromTo(pointcloud, move_from_point, move_to_point);

	viewerembeded->UpdateCloudViewer(pointcloud);

}

void MainUI::ButtonSetCloudAlignAxisPressed()
{
	cout << "call ButtonSetCloudAlignAxisPressed()" << endl;
	if (last_select_item_index == -1)
	{
		QMessageBox::information(0, QString("ButtonSetCloudAlignAxisPressed"),
			QString("No Item selected"), QMessageBox::Ok);
		return;
	}
	ObjectTransformationData *item_transform_data = dataprocess->items[last_select_item_index];
	cout << endl;
	cout << "major_vector " << item_transform_data->transform->major_vector << endl;
	cout << "minor_vector " << item_transform_data->transform->minor_vector << endl;
	cout << "middle_vector " << item_transform_data->transform->middle_vector << endl;
	
	PointCloudXYZRGB::Ptr pointcloud = item_transform_data->object_pointcloud;

	Eigen::Matrix<float, 1, 3>  floor_plane_normal_vector, target_plane_normal_vector;
	floor_plane_normal_vector[0] = item_transform_data->transform->major_vector(0);
	floor_plane_normal_vector[1] = item_transform_data->transform->major_vector(1);
	floor_plane_normal_vector[2] = item_transform_data->transform->major_vector(2);


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

	dataprocess->RotatePointCloudAlignAxis(pointcloud,
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
	/*ui->in_posobb_1->setText(
		QString::number(dataprocess->planeseg->transformextract->position_OBB.x));
	ui->in_posobb_2->setText(
		QString::number(dataprocess->planeseg->transformextract->position_OBB.y));
	ui->in_posobb_3->setText(
		QString::number(dataprocess->planeseg->transformextract->position_OBB.z));*/

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

	dataprocess->RotatePointCloudAlignAxis(dataprocess->GetCurrentDisplayPointCloud(),
		floor_plane_normal_vector, target_plane_normal_vector);


	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetCurrentDisplayPointCloud());
	viewerwindow->ClearShapeWindowCloudViewer();
	RadioButtonAxisONSelected();

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
	dataprocess->StoreLastedOperationCloud(dataprocess->GetCurrentDisplayPointCloud());
	PointCloudXYZRGB::Ptr pointcloud_display = dataprocess->GetCurrentDisplayPointCloud();
	PointCloudXYZRGB::Ptr pointcloud_for_extract_cluster(new PointCloudXYZRGB);

	pcl::copyPointCloud(*pointcloud_display, *pointcloud_for_extract_cluster);
	


	double cluster_tolerance = ui->in_clusterextract_tolerance->text().toDouble();
	int cluster_min_percentage = ui->in_clusterextract_min->text().toInt();
	int cluster_max_percentage = ui->in_clusterextract_max->text().toInt();
	int cluster_min_size = cluster_min_percentage*pointcloudsize / 100;
	int cluster_max_size = cluster_max_percentage*pointcloudsize / 100;

	

	dataprocess->clusterextract->SetClusterExtractValue(cluster_tolerance, cluster_min_size, cluster_max_size);
	dataprocess->clusterextract->ExtractCluster(pointcloud_for_extract_cluster);

	viewerwindow->AddPlanarAtOrigin(0.5, 0.5, 1.0, 1.0, 1.0, "planeorigin");


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
	/*viewerwindow->AddSphereWindowCloudViewer(obj->transform->position_OBB, 0.01,
		1.0, 0, 0, " obb" + id_name);
	viewerwindow->AddTextWindowCloudViewer(obj->transform->position_OBB, 0.01,
		1.0, 1.0, 1.0, "obb" + id_name, " text obb" + id_name);*/

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
	//extract data of each item TO dataprocess->container & items[]
	//show min max center point of each item
	//update it to ui
	ButtonClearAllItemPressed();

	dataprocess->SeparateContainerAndItems(dataprocess->clusterextract->GetExtractCluster());
	
	dataprocess->CalculateContainerTransformation();
	ShowMinMaxCenterPoint(dataprocess->container, "container");
	WriteContainerDataToUI(
		dataprocess->container->x_length_mm, dataprocess->container->y_length_mm, dataprocess->container->z_length_mm,
		dataprocess->container->transform->min3d_point, dataprocess->container->transform->max3d_point);


	dataprocess->CalculateItemsTransformation();
	for (int i = 0; i < dataprocess->items.size();i++)
	{
		ShowMinMaxCenterPoint(dataprocess->items[i], "item_" + to_string(i+1));
		WriteItemDataToUI(i+1,
			dataprocess->items[i]->x_length_mm,dataprocess->items[i]->y_length_mm,dataprocess->items[i]->z_length_mm,
			dataprocess->items[i]->object_pointcloud->size(),
			dataprocess->items[i]->transform->min3d_point,
			dataprocess->items[i]->transform->max3d_point,
			 "-"
			);
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
	//cout << "call PressedTreeItem()" << endl;

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


	if (current_select_item)
	{
		//hilight item clicked
		//item->setBackgroundColor(0, QColor(200, 200, 200));
		current_select_item->setBackgroundColor(1, QColor(200, 200, 200));
		current_select_item->setBackgroundColor(2, QColor(200, 200, 200));
		current_select_item->setBackgroundColor(3, QColor(200, 200, 200));
		current_select_item->setBackgroundColor(4, QColor(200, 200, 200));

		last_select_item_index = ui->treeWidget->currentIndex().row();



		//cout << "select " << last_select_item_index << endl;
		//show seleted cloud
		//program->ShowSelectedListedCloudIndex(last_select_item_index);


		PointCloudXYZRGB::Ptr pointcloud = dataprocess->items[last_select_item_index]->object_pointcloud;
		viewerembeded->UpdateCloudViewer(pointcloud); 
	}
};
void MainUI::ButtonLoadPointCloudToListPressed()
{
	cout << "call ButtonLoadPointCloudToListPressed()" << endl;

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Load pcd/ly"), POINTCLOUD_DIR, tr("point cloud (*.pcd *.ply)"));
	if (filename.trimmed().isEmpty()) return;



	PointCloudXYZRGB::Ptr cloud = dataprocess->LoadPcdFileToPointCloudVariable(filename.toStdString());
	if (cloud == NULL) return;
	//copy cloud to dataprocess->items[i]
	dataprocess->AddLoadPointCloudToItems(cloud);

	int pointcloud_number=ui->treeWidget->topLevelItemCount()+1;
	
	QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget); // at this  point  topLevelItemCount() is increase
	item->setText(0, QString::number(pointcloud_number));
	item->setText(4, QString::number(cloud->size()));
	item->setText(COLUMN_FILENAME, filename);
	
	item->setTextAlignment(0, Qt::AlignHCenter);
	for (int j = 1; j < 5; j++)
	{
		item->setTextAlignment(j, Qt::AlignRight);
	}
	for (int j = 5; j < 13; j++)
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
	last_selected_item->setText(COLUMN_FILENAME, filename);


}


void MainUI::ButtonAlignAllItemAxisPressed()
{
	cout << "call ButtonAlignAllItemAxisPressed()" << endl;
	// align at corner
	// align vector to paralel xyz axis
	int current_select_index = last_select_item_index;
	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		last_select_item_index = i;
		ButtonSetCloudCornerPressed();
		ButtonSetCloudAlignAxisPressed();
	}
	dataprocess->isSetAlignCorner = true;

	PressedTreeItem(ui->treeWidget->topLevelItem(current_select_index));

}
void MainUI::ButtonSaveAllItemPointcloudToPcdPressed()
{
	cout << "call ButtonSaveAllItemPointcloudToPcdPressed()" << endl;

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save pcd"), POINTCLOUD_DIR);
	if (filename.trimmed().isEmpty()) return;

	

	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		string eachfilename = filename.toStdString() + to_string(i + 1)+".pcd";
		dataprocess->SavePointCloud(eachfilename,
			dataprocess->items[i]->object_pointcloud);

		cout << eachfilename << endl;

		QTreeWidgetItem* item = ui->treeWidget->topLevelItem(i);
		item->setText(COLUMN_FILENAME, QString::fromStdString(eachfilename));
	}


}

void MainUI::WriteContainerDataToUI(
	int bin_x_dim, int bin_y_dim, int bin_z_dim, 
	PointTypeXYZRGB bin_min_pos, PointTypeXYZRGB bin_max_pos)
{
	ui->in_bin_x_dim->setText(QString::number(bin_x_dim));
	ui->in_bin_y_dim->setText(QString::number(bin_y_dim));
	ui->in_bin_z_dim->setText(QString::number(bin_z_dim));
	ui->in_min_pos_container_1->setText(QString::number(bin_min_pos.x));
	ui->in_min_pos_container_2->setText(QString::number(bin_min_pos.y));
	ui->in_min_pos_container_3->setText(QString::number(bin_min_pos.z));
	ui->in_max_pos_container_1->setText(QString::number(bin_max_pos.x));
	ui->in_max_pos_container_2->setText(QString::number(bin_max_pos.y));
	ui->in_max_pos_container_3->setText(QString::number(bin_max_pos.z));
}

void MainUI::WriteItemDataToUI(int index,
	int box_x_dim, int box_y_dim, int box_z_dim, size_t cloud_size, 
	PointTypeXYZRGB box_min_pos, PointTypeXYZRGB box_max_pos,
	string file_name)
{

	QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidget);

	item->setText(0, QString::number(index));
	item->setText(1, QString::number(box_x_dim));
	item->setText(2, QString::number(box_y_dim));
	item->setText(3, QString::number(box_z_dim));
	item->setText(4, QString::number(cloud_size));
	item->setText(5, QString::number(box_min_pos.x));
	item->setText(6, QString::number(box_min_pos.y));
	item->setText(7, QString::number(box_min_pos.z));
	item->setText(8, QString::number(box_max_pos.x));
	item->setText(9, QString::number(box_max_pos.y));
	item->setText(10, QString::number(box_max_pos.z));
	//item->setText(11, QString::number(rotate_case));
	//item->setText(12, QString::number(order));
	item->setText(COLUMN_FILENAME, QString::fromStdString(file_name));

	item->setTextAlignment(0, Qt::AlignHCenter);
	for (int j = 1; j < 5; j++)
	{
		item->setTextAlignment(j, Qt::AlignRight);
	}
	for (int j = 5; j < 13; j++)
	{
		item->setTextAlignment(j, Qt::AlignLeft);
	}
	item->setTextAlignment(11, Qt::AlignHCenter);

	item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);

	ui->treeWidget->addTopLevelItem(item);
}


void MainUI::ButtonLoadAllItemsTextToUIPressed()
{
	cout << "call ButtonLoadAllItemsTextToUIPressed()" << endl;
	//clear ui
	//clear dataprocess->container & items[]
	//new PointCloudBPPTextListIO in order to clear variable value
	//load from txt file
	//update it to ui
	//add it in dataprocess
	//set dataprocess->isSetAlignCorner = true; -> align cornor again min_point will change to 0,...

	

	QString filename = QFileDialog::getOpenFileName(this,
		tr("Load container & items list file"), POINTCLOUD_DIR, tr("text file (*.txt)"));
	if (filename.trimmed().isEmpty()) return;

	Call_LoadAllItemsTextToUI(filename.toStdString());


}
void MainUI::Call_LoadAllItemsTextToUI(string filename)
{
	int total_items;

	ButtonClearAllItemPressed();

	PointCloudBPPTextListIO *txt_items = new PointCloudBPPTextListIO();

	total_items = txt_items->ReadPointCloudListForBPP(filename);


	dataprocess->container = new ObjectTransformationData();
	dataprocess->container->SetLengthMM(txt_items->bin_x_dim, txt_items->bin_y_dim, txt_items->bin_z_dim);
	dataprocess->container->SetTransformMinMaxPos(txt_items->bin_min_pos, txt_items->bin_max_pos);
	dataprocess->container->SetTransformVector(txt_items->bin_major_vector, txt_items->bin_middle_vector, txt_items->bin_minor_vector);

	WriteContainerDataToUI(txt_items->bin_x_dim, txt_items->bin_y_dim, txt_items->bin_z_dim,
		txt_items->bin_min_pos, txt_items->bin_max_pos);

	for (int i = 0; i < total_items; i++)
	{
		PointCloudXYZRGB::Ptr cloud = dataprocess->LoadPcdFileToPointCloudVariable(
			txt_items->array_pcd_filename[i]);
		if (cloud == NULL) return;

		//add item to dataprocess
		//dataprocess->AddLoadPointCloudToItems(cloud);

		ObjectTransformationData *item = new ObjectTransformationData();
		pcl::copyPointCloud(*cloud, *item->object_pointcloud);
		item->SetLengthMM(txt_items->items_x_dim[i], txt_items->items_y_dim[i], txt_items->items_z_dim[i]);
		item->SetTransformMinMaxPos(txt_items->items_min_pos[i], txt_items->items_max_pos[i]);
		item->SetTransformVector(txt_items->items_major_vector[i], txt_items->items_middle_vector[i], txt_items->items_minor_vector[i]);
		dataprocess->items.push_back(item);
		cout << "dataprocess ->items.size()=" << dataprocess->items.size() << endl;

		WriteItemDataToUI(i + 1,
			txt_items->items_x_dim[i], txt_items->items_y_dim[i], txt_items->items_z_dim[i],
			cloud->size(), txt_items->items_min_pos[i], txt_items->items_max_pos[i],
			txt_items->array_pcd_filename[i]
			);
	}

	dataprocess->isSetAlignCorner = true;

	delete txt_items;
}


void MainUI::ButtonSaveAllItemsUIToTextPressed()
{
	cout << "call ButtonSaveAllItemsUIToTextPressed()" << endl;

	int ui_total_boxes = ui->treeWidget->topLevelItemCount();

	if (ui_total_boxes != dataprocess->items.size())
	{
		QMessageBox::information(0, QString("ButtonSaveAllItemsUIToTextPressed"), 
			QString("ui_total_boxes != dataprocess->items.size()"), QMessageBox::Ok);

		return;
	}

	//Do read value in ui not dataprocess->container/dataprocess->items[]
	//Because user may change some item value

	int bin_x_dim = ui->in_bin_x_dim->text().toInt();
	int bin_y_dim = ui->in_bin_y_dim->text().toInt();
	int bin_z_dim = ui->in_bin_z_dim->text().toInt();

	PointTypeXYZRGB bin_min_pos;
	bin_min_pos.x = ui->in_min_pos_container_1->text().toDouble();
	bin_min_pos.y = ui->in_min_pos_container_2->text().toDouble();
	bin_min_pos.z = ui->in_min_pos_container_3->text().toDouble();

	PointTypeXYZRGB bin_max_pos;
	bin_max_pos.x = ui->in_max_pos_container_1->text().toDouble();
	bin_max_pos.y = ui->in_max_pos_container_2->text().toDouble();
	bin_max_pos.z = ui->in_max_pos_container_3->text().toDouble();

	Eigen::Vector3f bin_major_vector = dataprocess->container->transform->major_vector;
	Eigen::Vector3f bin_middle_vector = dataprocess->container->transform->middle_vector;
	Eigen::Vector3f bin_minor_vector = dataprocess->container->transform->minor_vector;


	vector<string> array_pcd_filename;
	vector<int> items_x_dim, items_y_dim, items_z_dim;
	vector<PointTypeXYZRGB> items_min_pos, items_max_pos;
	vector<Eigen::Vector3f> items_major_vector, items_middle_vector, items_minor_vector;


	for (int i = 0; i < ui_total_boxes; i++)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		if (item->text(COLUMN_FILENAME) == "")
		{
			//cout << i << " no filename" << endl;
			QMessageBox::information(0, QString("ButtonSaveAllItemsUIToTextPressed"), 
				QString("Each item Pcd File has not been saved"), QMessageBox::Ok);
			return;
		}
		else
		{
			array_pcd_filename.push_back(item->text(COLUMN_FILENAME).toStdString());
		}

		if (item->text(1) == "")
		{
			QMessageBox::information(0, QString("ButtonSaveAllItemsUIToTextPressed"), 
				QString("Pointcloud did not calculate dimension yet"), QMessageBox::Ok);
			return;
		}
		else
		{
			items_x_dim.push_back(item->text(1).toInt());
			items_y_dim.push_back(item->text(2).toInt());
			items_z_dim.push_back(item->text(3).toInt());

			PointTypeXYZRGB item_min_pos;
			item_min_pos.x = item->text(5).toDouble();
			item_min_pos.y = item->text(6).toDouble();
			item_min_pos.z = item->text(7).toDouble();
			items_min_pos.push_back(item_min_pos);

			PointTypeXYZRGB item_max_pos;
			item_max_pos.x = item->text(8).toDouble();
			item_max_pos.y = item->text(9).toDouble();
			item_max_pos.z = item->text(10).toDouble();
			items_max_pos.push_back(item_max_pos);

			items_major_vector.push_back(dataprocess->items[i]->transform->major_vector);
			items_middle_vector.push_back(dataprocess->items[i]->transform->middle_vector);
			items_minor_vector.push_back(dataprocess->items[i]->transform->minor_vector);

			cout << "item_min_pos=" << item_min_pos << endl;
		}
	}

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save txt"), POINTCLOUD_DIR, tr("list of pcd (*.txt)"));

	if (filename.trimmed().isEmpty()) return;

	

	dataprocess->pointcloudbpptext->WritePointCloudListForBPP(
		filename.toStdString(), ui_total_boxes,
		bin_x_dim, bin_y_dim, bin_z_dim,
		bin_min_pos, bin_max_pos,
		bin_major_vector, bin_middle_vector, bin_minor_vector,
		array_pcd_filename,
		items_x_dim, items_y_dim, items_z_dim,
		items_min_pos, items_max_pos,
		items_major_vector, items_middle_vector, items_minor_vector
		);


}
void MainUI::ButtonRemoveItemPressed()
{
	cout << "call ButtonRemoveItemPressed() " << last_select_item_index << endl;
	//remove item from ui
	//remove item from dataprocess->items[i]
	//re run index number
	//update viewer embeded to next pointcloud

	QTreeWidgetItem* last_selected_item = ui->treeWidget->topLevelItem(last_select_item_index);
	delete last_selected_item;

	// Deletes the element at (0+last_select_item_index)
	dataprocess->items.erase(dataprocess->items.begin() + last_select_item_index);

	//rerun index number
	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; ++i)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		item->setText(0, QString::number(i + 1));

	}

	//if delete last item:next select item = deleteitem index-1 
	if (last_select_item_index == total_boxes)
	{
		last_select_item_index--;
	}
	//cout << "last_select_item_index=" << last_select_item_index << endl;



	if (dataprocess->items.size() != total_boxes)
	{
		QMessageBox::information(0, QString("ButtonRemoveItemPressed"), QString("dataprocess->items.size() != total_boxes"), QMessageBox::Ok);
		return;
	}

	//show next pointcloud in viewer embeded
	if (last_select_item_index >= 0)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(last_select_item_index);
		PressedTreeItem(item);
	}
	else
	{
		viewerembeded->ClearPointCloudEmbededCloudViewer();
	}

}
void MainUI::ButtonClearAllItemPressed()
{
	cout << "call ButtonClearAllItemPressed()" << endl;
	//remove container and item from ui
	//remove container and item from dataprocess->items[i]	
	//clear viewer embeded
	//set last_select_item_index to select nothing

	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(0);
		delete item;

		dataprocess->items[i] = NULL;
		delete dataprocess->items[i];
	}

	int total_order = ui->treeWidgetSorting->topLevelItemCount();
	for (int i = 0; i < total_order; i++)
	{
		QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(0);
		delete item;
	}

	PointTypeXYZRGB point;
	WriteContainerDataToUI(0, 0, 0, point, point);

	// Deletes all element 
	dataprocess->items.clear();

	dataprocess->container = NULL;
	delete dataprocess->container;

	viewerembeded->ClearPointCloudEmbededCloudViewer();
	last_select_item_index = -1;

}
void MainUI::ButtonCalculateBinPackingPressed()
{
	cout << "call ButtonCalculateBinPackingPressed()" << endl;
	//read data from ui, create new variable
	//calculate rotation case
	//save it in ui and dataprocess->items[i]
	//reorder
	//save new order to dataprocess->items[i]
	//show new order at ui

	viewerwindow->ClearPointCloudWindowCloudViewer();
	viewerwindow->ClearShapeWindowCloudViewer();

	//check number of item in ui = item in dataprocess
	int total_boxes = ui->treeWidget->topLevelItemCount();
	if (total_boxes != dataprocess->items.size())
	{
		cout << "total_boxes = " << total_boxes << endl;
		cout << "dataprocess->items.size() = " << dataprocess->items.size() << endl;
		cout << "total_boxes != dataprocess->items.size()  RETURN FUNCTION" << endl;
		return;
	}

	//check that pointcloud already arrange at 0,0,0
	if (dataprocess->isSetAlignCorner == false)
	{
		QMessageBox::information(0, QString("Point cloud not corner"), QString("Please Set Pointcloud at corner first"), QMessageBox::Ok);
		return;
	}

	// new variable and get value from ui
	int *boxes_r = new int[total_boxes];
	int *boxes_g = new int[total_boxes];
	int *boxes_b = new int[total_boxes];

	int *boxes_x_orient = new int[total_boxes];
	int *boxes_y_orient = new int[total_boxes];
	int *boxes_z_orient = new int[total_boxes];

	int *boxes_x_pos = new int[total_boxes];
	int *boxes_y_pos = new int[total_boxes];
	int *boxes_z_pos = new int[total_boxes];

	int *boxes_x_dim = new int[total_boxes];
	int *boxes_y_dim = new int[total_boxes];
	int *boxes_z_dim = new int[total_boxes];

	int *boxes_bin_num = new int[total_boxes];
	int *boxes_item_num = new int[total_boxes];
	int *boxes_item_order = new int[total_boxes];

	int *boxes_item_rotation = new int[total_boxes];

	for (int i = 0; i < total_boxes; i++)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		boxes_x_dim[i] = item->text(1).toInt();
		boxes_y_dim[i] = item->text(2).toInt();
		boxes_z_dim[i] = item->text(3).toInt();

		boxes_x_orient[i] = 0;
		boxes_y_orient[i] = 0;
		boxes_z_orient[i] = 0;

		boxes_x_pos[i] = 0;
		boxes_y_pos[i] = 0;
		boxes_z_pos[i] = 0;

		boxes_bin_num[i] = 0;
		boxes_item_num[i] = i;
		boxes_item_rotation[i] = -1;

	}

	//just print for debug
	for (int i = 0; i < total_boxes; i++)
	{
		cout
			<< "No." << boxes_item_num[i] << ", "
			<< "bin_num:" << boxes_bin_num[i] << ", "
			<< "whd:" << boxes_x_dim[i] << "x" << boxes_y_dim[i] << "x" << boxes_z_dim[i] << ", "
			<< "pos:" << boxes_x_pos[i] << "," << boxes_y_pos[i] << "," << boxes_z_pos[i] << ", "
			<< "orient:" << boxes_x_orient[i] << "," << boxes_y_orient[i] << "," << boxes_z_orient[i] << ", "
			<< endl;
	}

	// new class, in order to when make a 2nd caculate the data in class will be refresh
	// call calculate bin packing, 
	// this class also copy address of input to it's class variable
	dataprocess->bpp = new CalculateBppErhan(
		total_boxes,
		ui->in_bin_x_dim->text().toDouble(),
		ui->in_bin_y_dim->text().toDouble(),
		ui->in_bin_z_dim->text().toDouble(),
		boxes_x_dim, boxes_y_dim, boxes_z_dim,
		boxes_x_pos, boxes_y_pos, boxes_z_pos,
		boxes_x_orient, boxes_y_orient, boxes_z_orient,
		boxes_r, boxes_g, boxes_b,
		boxes_bin_num, boxes_item_num, boxes_item_order, 
		boxes_item_rotation);

	dataprocess->bpp->CalculateBinpack();
	dataprocess->bpp->CalculateRotationCase();
	//dataprocess->bpp->SortBoxesOrder();

	//just print for debug
	for (int i = 0; i < total_boxes; i++)
	{
		cout
			<< "item_num." << boxes_item_num[i] << ", "
			<< "bin_num:" << boxes_bin_num[i] << ", "
			<< "whd:" << boxes_x_dim[i] << "x" << boxes_y_dim[i] << "x" << boxes_z_dim[i] << ", "
			<< "pos:" << boxes_x_pos[i] << "," << boxes_y_pos[i] << "," << boxes_z_pos[i] << ", "
			<< "rotation:" << boxes_item_rotation[i] << ", "
			<< endl;
	}

	// write rotation_case, target_position, target_orientation to dataprocess
	int item_packed_count = 0;
	for (int i = 0; i < total_boxes; i++)
	{
		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		item->setText(11, QString::number(boxes_item_rotation[i]));

		ObjectTransformationData *itm = dataprocess->items[i];
		itm->rotation_case = boxes_item_rotation[i];
		if (boxes_bin_num[i] == 1)
		{
			item_packed_count++;
			//do data in ui and data in items[i] is equal??
			itm->target_position.x = boxes_x_pos[i] * 0.001;
			itm->target_position.y = boxes_y_pos[i] * 0.001;
			itm->target_position.z = boxes_z_pos[i] * 0.001;

			itm->target_orientation.x = boxes_x_orient[i];
			itm->target_orientation.y = boxes_y_orient[i];
			itm->target_orientation.z = boxes_z_orient[i];
		}
	}

	cout << endl;
	cout << "packed " << item_packed_count << "/" << total_boxes << endl;
	cout << endl;
	cout << "sort box order" << endl;
	//sort packing box order from position 0,0,0 to max,max,max
	dataprocess->bpp->SortBoxesOrder();

	//just print for debug
	for (int i = 0; i < total_boxes; i++)
	{
		cout
			<< "item_num." << boxes_item_num[i] << ", "
			<< "item_order:" << boxes_item_order[i] << ", "
			<< "bin_num:" << boxes_bin_num[i] << ", "
			<< "whd:" << boxes_x_dim[i] << "x" << boxes_y_dim[i] << "x" << boxes_z_dim[i] << ", "
			<< "pos:" << boxes_x_pos[i] << "," << boxes_y_pos[i] << "," << boxes_z_pos[i] << ", "
			<< "rotation:" << boxes_item_rotation[i] 
			<< endl;
	}

	// write box packing_order to dataprocess & ui
	// current array is not arrange by item index
	int *item_number_of_order_index = new int[item_packed_count];
	for (int i = 0; i < total_boxes; i++)
	{
		int box_item_index = boxes_item_num[i];//0 -> size()-1
		int box_order_index = boxes_item_order[i];//0 -> size()-1

		ObjectTransformationData *itm = dataprocess->items[box_item_index];
		itm->packing_order = box_order_index;

		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(box_item_index);
		item->setText(12, QString::number(box_order_index));

		item_number_of_order_index[box_order_index] = box_item_index;
	}
	cout << endl;

	//write order of packing item in uiSorting
	for (int i = 0; i < item_packed_count; i++)
	{
		ObjectTransformationData *itm = dataprocess->items[item_number_of_order_index[i]];


		QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidgetSorting); // at this  point  topLevelItemCount() is increase
		item->setText(0, QString::number(i+1));  //order number
		item->setText(1, QString::number(itm->target_position.x)); //item packing position
		item->setText(2, QString::number(itm->target_position.y));
		item->setText(3, QString::number(itm->target_position.z));
		item->setText(4, QString::number(item_number_of_order_index[i]+1)); // item number

/*		item->setTextAlignment(0, Qt::AlignHCenter);
		for (int j = 1; j < 5; j++)
		{
			item->setTextAlignment(j, Qt::AlignRight);
		}
		for (int j = 5; j < 13; j++)
		{
			item->setTextAlignment(j, Qt::AlignLeft);
		}*/

		item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);

		ui->treeWidgetSorting->addTopLevelItem(item);


	}
	


	delete dataprocess->bpp;
}

void MainUI::ButtonShowProjectionInputPositionPressed()
{
	cout << "call ButtonShowProjectionInputPositionPressed()" << endl;
	//draw box / or rectangle for confirm items and box position

	
	current_display_packing_number = 0;

	viewerwindow->ClearPointCloudWindowCloudViewer();
	viewerwindow->ClearShapeWindowCloudViewer();


	//rectangle of container
	PointTypeXYZRGB draw_container_pos = dataprocess->container->transform->min3d_point;


	viewerwindow->Add2DRectangle(
		draw_container_pos,
		dataprocess->container->x_length, dataprocess->container->z_length,
		1.0, 1.0, 1.0, "container_rectangle");

	//rectangle for items
	int total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < total_boxes; i++)
	{
		PointTypeXYZRGB draw_rec_pos = dataprocess->items[i]->transform->min3d_point;
		
		string item_rec_name = "item_rectangle" + to_string(i);
		string item_text_name = "item_text" + to_string(i);
		string clouditem_name = "clouditem" + to_string(i);
		viewerwindow->Add2DRectangle(
			draw_rec_pos,
			dataprocess->items[i]->x_length, dataprocess->items[i]->z_length,
			1.0, 1.0, 1.0, item_rec_name);

		//viewerwindow->AddTextWindowCloudViewer(dataprocess->items[i]->transform->mass_center_point,
		//	0.05, 1.0, 0, 0, to_string(i+1), item_text_name);

		PointCloudXYZRGB::Ptr item_pointcloud;
		item_pointcloud.reset(new PointCloudXYZRGB);
		pcl::copyPointCloud(*dataprocess->items[i]->object_pointcloud, *item_pointcloud);

		dataprocess->TranslatePointCloud(item_pointcloud,
			draw_rec_pos.x, draw_rec_pos.y, draw_rec_pos.z);


		//cout << clouditem_name << endl;
		//cout << item_pointcloud->size() << endl;
		//cout << draw_rec_pos << endl;
		viewerwindow->AddPointCloudItem(item_pointcloud, clouditem_name);

	}

	viewerwindow->window_view->spinOnce();



}
void MainUI::ButtonShowTestCubePressed()
{

}


void MainUI::ButtonShowTestRectanglePositionPressed()
{
	viewerwindow->ClearPointCloudWindowCloudViewer();
	viewerwindow->ClearShapeWindowCloudViewer();

	float x_length = 0.142;
	float z_length = 0.123;

	float x_space_length = 0.2;
	float z_space_length = 0.2;
	int num_x = 3;
	int num_z = 3;
	int dx = -3;
	int dz = -2;

	int index = 1;




	for (int i = dx; i < num_x; i++)
	{
		for (int j = dz; j < num_z; j++)
		{

			PointTypeXYZRGB draw_rec_pos;
			draw_rec_pos.x = x_space_length*i;
			draw_rec_pos.y = 0;
			draw_rec_pos.z = z_space_length*j;
	
		
			string item_rec_name = "item_rectangle" + index;
			index++;
			cout << item_rec_name << " " << draw_rec_pos << endl;

			viewerwindow->Add2DRectangle(
				draw_rec_pos,
				x_length, z_length,
				1.0, 1.0, 1.0, item_rec_name);
		}

	}

	viewerwindow->window_view->spinOnce();
}
void MainUI::ButtonSetContainerItemsYzeroPressed()
{
	int ui_total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < ui_total_boxes; i++)
	{
		ObjectTransformationData *itm = dataprocess->items[i];
		itm->transform->min3d_point.y = 0;
		itm->transform->mass_center_point.y = 0;

		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		item->setText(6, QString::number(itm->transform->min3d_point.y));
	}

	dataprocess->container->transform->min3d_point.y = 0;
	dataprocess->container->transform->mass_center_point.y = 0;
	ui->in_min_pos_container_2->setText(
		QString::number(dataprocess->container->transform->min3d_point.y));
}
void MainUI::ButtonUpdateContainerItemsToDataprocessPressed()
{
	int ui_total_boxes = ui->treeWidget->topLevelItemCount();
	for (int i = 0; i < ui_total_boxes; i++)
	{


		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(i);
		int item_x_dim = item->text(1).toInt();
		int item_y_dim = item->text(2).toInt();
		int item_z_dim = item->text(3).toInt();

		PointTypeXYZRGB item_min_pos, item_max_pos;
		item_min_pos.x = item->text(5).toDouble();
		item_min_pos.y = item->text(6).toDouble();
		item_min_pos.z = item->text(7).toDouble();

		item_max_pos.x = item->text(8).toDouble();
		item_max_pos.y = item->text(9).toDouble();
		item_max_pos.z = item->text(10).toDouble();

		ObjectTransformationData *itm = dataprocess->items[i];
		itm->SetLengthMM(item_x_dim, item_y_dim, item_z_dim);
		itm->SetTransformMinMaxPos(item_min_pos, item_max_pos);


	}


	int bin_x_dim = ui->in_bin_x_dim->text().toInt();
	int bin_y_dim = ui->in_bin_y_dim->text().toInt();
	int bin_z_dim = ui->in_bin_z_dim->text().toInt();

	PointTypeXYZRGB bin_min_pos;
	bin_min_pos.x = ui->in_min_pos_container_1->text().toDouble();
	bin_min_pos.y = ui->in_min_pos_container_2->text().toDouble();
	bin_min_pos.z = ui->in_min_pos_container_3->text().toDouble();

	PointTypeXYZRGB bin_max_pos;
	bin_max_pos.x = ui->in_max_pos_container_1->text().toDouble();
	bin_max_pos.y = ui->in_max_pos_container_2->text().toDouble();
	bin_max_pos.z = ui->in_max_pos_container_3->text().toDouble();

	dataprocess->container->SetLengthMM(bin_x_dim, bin_y_dim, bin_z_dim);
	dataprocess->container->SetTransformMinMaxPos(bin_min_pos, bin_max_pos);

}




void MainUI::ButtonShowPackingTargetPressed()
{
	cout << "call ButtonShowPackingTargetPressed()" << endl;

	ui->radioButton_packing_1->setChecked(true);
	ui->radioButton_packing_2->setChecked(false);
	ui->radioButton_packing_3->setChecked(false);
	current_display_packing_number = 0;

	viewerwindow->ClearPointCloudWindowCloudViewer();
	viewerwindow->ClearShapeWindowCloudViewer();
	
	//hilight circle at each item (input_position)
	int total_order = ui->treeWidgetSorting->topLevelItemCount();
	for (int i = 0; i < total_order; i++)
	{
		QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(i);
		int dataprocess_number = item->text(4).toInt();
		int dataprocess_index = dataprocess_number - 1;
		cout << "number=" << dataprocess_number << ", order_index[" << i << "] dataprocess_index=" << dataprocess_index << endl;
		viewerwindow->ShowBinPackingTarget(dataprocess->container, dataprocess->items[dataprocess_index], dataprocess_index);
	}
}
void MainUI::ButtonShowPackingIndicatePressed()
{
	cout << "call ButtonShowPackingIndicatePressed() " << dataprocess->items.size() <<endl;
	ui->radioButton_packing_1->setChecked(false);
	ui->radioButton_packing_2->setChecked(true);
	ui->radioButton_packing_3->setChecked(false);
	current_display_packing_number = 0;

	viewerwindow->ClearPointCloudWindowCloudViewer();
	viewerwindow->ClearShapeWindowCloudViewer();

	int total_order = ui->treeWidgetSorting->topLevelItemCount();
	for (int i = 0; i < total_order; i++)
	{
		QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(i);
		int dataprocess_number = item->text(4).toInt();
		int dataprocess_index = dataprocess_number - 1;
		cout << "number=" << dataprocess_number << ", order_index[" << i << "] dataprocess_index=" << dataprocess_index << endl;
		viewerwindow->ShowBinpackingIndication(dataprocess->container, dataprocess->items[dataprocess_index], dataprocess_index);
	}
	

}
void MainUI::ButtonShowPackingAnimationPressed()
{
	cout << "call ButtonShowPackingAnimationPressed()" << endl;
	ui->radioButton_packing_1->setChecked(false);
	ui->radioButton_packing_2->setChecked(false);
	ui->radioButton_packing_3->setChecked(true);
/*	
	current_display_packing_number = 0;

ObjectTransformationData *test_container = new ObjectTransformationData();
	test_container->transform->min3d_point.x = -0.353263;
	test_container->transform->min3d_point.y = 0.0587912;
	test_container->transform->min3d_point.z = -0.0622235;

	ObjectTransformationData *test_item = new ObjectTransformationData();
	test_item->transform->min3d_point.x = 0.235024;
	test_item->transform->min3d_point.y = 0.00211017;
	test_item->transform->min3d_point.z = 0.102774;

	test_item->x_length = 0.196;
	test_item->y_length = 0.034;
	test_item->z_length = 0.115;
	
	test_item->target_position.x = 0.238;
	test_item->target_position.y = 0;
	test_item->target_position.z = 0;

	test_item->target_orientation.x = 34;
	test_item->target_orientation.y = 115;
	test_item->target_orientation.z = 196;

	test_item->rotation_case = 5;
	

	viewerwindow->ShowBinpackingAnimation(test_container, test_item);
	*/
	int item_number = current_display_packing_number;
	int item_index = item_number-1;
	
	QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(item_index);
	int dataprocess_number = item->text(4).toInt();
	int dataprocess_index = dataprocess_number - 1;

	cout << "number=" << dataprocess_number << ", order_index[" << item_index << "] dataprocess_index=" << dataprocess_index << endl;
	if (item_index >= 0)
	{ 
		viewerwindow->ShowBinpackingAnimation(dataprocess->container, dataprocess->items[dataprocess_index]);
	}


}

void MainUI::ButtonShowZeroPackingPressed()
{
	//cout << "call ButtonShowZeroPackingPressed()" << endl;
	//viewerwindow->AddPlanarAtOrigin(116.7/200,61.1/200,1.0,1.0,0.0,"realsizetable");

	if (last_select_sorting_index>=0)
	{ 
		//clear hilight
		QTreeWidgetItem* last_selected_item = ui->treeWidgetSorting->topLevelItem(last_select_sorting_index);
		//last_selected_item->setBackgroundColor(0, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(1, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(2, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(3, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(4, QColor(255, 255, 255));
	}

	last_select_sorting_index = -1;
	current_display_packing_number = 0;
	ui->bt_order_one->setText(QString::number(current_display_packing_number));
	
	viewerwindow->ClearPointCloudWindowCloudViewer();
	viewerwindow->ClearShapeWindowCloudViewer();

}
void MainUI::ShowPackingCurrentOrder()
{


	int total_order = ui->treeWidgetSorting->topLevelItemCount();
	int item_number = current_display_packing_number;
	int item_index = item_number - 1;

	if (ui->checkBox_clear_previous->isChecked())
	{
		viewerwindow->ClearPointCloudWindowCloudViewer();
		viewerwindow->ClearShapeWindowCloudViewer();
	}

	if (item_index >= 0 && item_index < total_order)
	{
		QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(item_index);
		//ui->treeWidgetSorting->setCurrentItem(item);
		//PressedTreeSorting(item);

		int dataprocess_number = item->text(4).toInt();
		int dataprocess_index = dataprocess_number - 1;
		cout << "number = " << dataprocess_number << ", order_index[" << item_index << "] dataprocess_index = " << dataprocess_index << endl;

		if (ui->radioButton_packing_1->isChecked())
		{
			viewerwindow->ShowBinPackingTarget(dataprocess->container, dataprocess->items[dataprocess_index], dataprocess_index);
		}
		else if (ui->radioButton_packing_2->isChecked())
		{
			viewerwindow->ShowBinpackingIndication(dataprocess->container, dataprocess->items[dataprocess_index], dataprocess_index);
		}
		else if (ui->radioButton_packing_3->isChecked())
		{
			viewerwindow->ShowBinpackingAnimation(dataprocess->container, dataprocess->items[dataprocess_index]);
		}
	}

}
void MainUI::ButtonShowPrevPackingPressed()
{
	//cout << "call ButtonShowPrevPackingPressed()" << endl;
	if (current_display_packing_number > 0)
	{
		current_display_packing_number--;
		ui->bt_order_one->setText(QString::number(current_display_packing_number));

		//ShowPackingCurrentOrder();
		int item_index = current_display_packing_number - 1;
		QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(item_index);
		ui->treeWidgetSorting->setCurrentItem(item);
		PressedTreeSorting(item);
	}

	if (current_display_packing_number == 0)// if has already hilight previous one
	{
		ButtonShowZeroPackingPressed();
	}



}
void MainUI::ButtonShowNextPackingPressed()
{
	//cout << "call ButtonShowNextPackingPressed()" << endl;
	if (current_display_packing_number < ui->treeWidgetSorting->topLevelItemCount())
	{
		current_display_packing_number++;
		ui->bt_order_one->setText(QString::number(current_display_packing_number));
		
		//ShowPackingCurrentOrder();
		int item_index = current_display_packing_number - 1;
		QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(item_index);
		ui->treeWidgetSorting->setCurrentItem(item);
		PressedTreeSorting(item);
	}
	else if (current_display_packing_number == ui->treeWidgetSorting->topLevelItemCount())// if has already hilight previous one
	{
		current_display_packing_number++;
		viewerwindow->ClearPointCloudWindowCloudViewer();
		viewerwindow->ClearShapeWindowCloudViewer();
	}

}


void MainUI::PressedTreeSorting(QTreeWidgetItem *current_select_item)
{
	if (last_select_sorting_index != -1)// if has already hilight previous one
	{
		//remove hilight last selected item
		QTreeWidgetItem* last_selected_item = ui->treeWidgetSorting->topLevelItem(last_select_sorting_index);
		//last_selected_item->setBackgroundColor(0, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(1, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(2, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(3, QColor(255, 255, 255));
		last_selected_item->setBackgroundColor(4, QColor(255, 255, 255));
	}


	if (current_select_item)
	{ 
		//hilight item clicked
		//item->setBackgroundColor(0, QColor(200, 200, 200));
		current_select_item->setBackgroundColor(1, QColor(200, 200, 200));
		current_select_item->setBackgroundColor(2, QColor(200, 200, 200));
		current_select_item->setBackgroundColor(3, QColor(200, 200, 200));
		current_select_item->setBackgroundColor(4, QColor(200, 200, 200));

		last_select_sorting_index = ui->treeWidgetSorting->currentIndex().row();

		current_display_packing_number = last_select_sorting_index + 1;
		ui->bt_order_one->setText(QString::number(current_display_packing_number));
		ShowPackingCurrentOrder();
	}

}

void MainUI::ButtonSaveBinPackingInfoPressed()
{
	int total_order = ui->treeWidgetSorting->topLevelItemCount();
	vector<int> item_index_s, packing_order_s, rotation_case_s;
	vector<PointTypeXYZRGB> target_position_s, target_orientation_s;

	for (int i = 0; i < total_order; i++)
	{
		QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(i);
		int item_number = item->text(4).toInt();
		int item_index = item_number - 1;

		ObjectTransformationData *itm = dataprocess->items[item_index];
		item_index_s.push_back(item_index);
		packing_order_s.push_back(itm->packing_order);
		rotation_case_s.push_back(itm->rotation_case);
		target_position_s.push_back(itm->target_position);
		target_orientation_s.push_back(itm->target_orientation);



	}

	QString filename = QFileDialog::getSaveFileName(this,
		tr("Save packing info"), POINTCLOUD_DIR, tr("packing info (*.txt)"));

	if (filename.trimmed().isEmpty()) return;


	dataprocess->bppresult->WriteBinPackingResult(filename.toStdString(), total_order,
		packing_order_s, item_index_s, rotation_case_s,
		target_position_s, target_orientation_s);

}
void MainUI::ButtonLoadBinPackingInfoPressed()
{
	QString filename = QFileDialog::getOpenFileName(this,
		tr("Load packing info"), POINTCLOUD_DIR, tr("packing info (*.txt)"));
	if (filename.trimmed().isEmpty()) return;

	Call_LoadBinPackingInfo(filename.toStdString());
}

void MainUI::Call_LoadBinPackingInfo(string filename)
{
	//clear order widget
	int treeWidgetSorting_size = ui->treeWidgetSorting->topLevelItemCount();
	for (int i = 0; i < treeWidgetSorting_size; i++)
	{
		QTreeWidgetItem *item = ui->treeWidgetSorting->topLevelItem(0);
		delete item;
	}

	BPPResultIO *txt_bppinfo = new BPPResultIO();
	int total_order = txt_bppinfo->ReadBinPackingResult(filename);

	//write data to data process and uiSorting
	int *item_number_of_order_index = new int[total_order];
	for (int i = 0; i < total_order; i++)
	{
		int item_index = txt_bppinfo->item_index[i];
		ObjectTransformationData *itm = dataprocess->items[item_index];
		itm->packing_order = txt_bppinfo->packing_order[i];
		itm->rotation_case = txt_bppinfo->rotation_case[i];
		itm->target_position = txt_bppinfo->target_position[i];
		itm->target_orientation = txt_bppinfo->target_orientation[i];

		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(item_index);
		item->setText(11, QString::number(itm->rotation_case));
		item->setText(12, QString::number(itm->packing_order));

		item_number_of_order_index[itm->packing_order] = item_index;
	}


	for (int i = 0; i < total_order; i++)
	{

		ObjectTransformationData *itm = dataprocess->items[item_number_of_order_index[i]];

		QTreeWidgetItem *item = new QTreeWidgetItem(ui->treeWidgetSorting); // at this  point  topLevelItemCount() is increase
		item->setText(0, QString::number(i+1));  //order index
		item->setText(1, QString::number(itm->target_position.x)); //item packing position
		item->setText(2, QString::number(itm->target_position.y));
		item->setText(3, QString::number(itm->target_position.z));
		item->setText(4, QString::number(item_number_of_order_index[i]+1)); // item index

		item->setFlags(Qt::ItemIsEditable | Qt::ItemIsEnabled);

		ui->treeWidgetSorting->addTopLevelItem(item);

	}

}

void MainUI::ButtonMoveUpPackingOrderPressed()
{
	QTreeWidgetItem* item = ui->treeWidgetSorting->currentItem();
	int row = ui->treeWidgetSorting->currentIndex().row();

	if (row > 0)
	{
		ui->treeWidgetSorting->takeTopLevelItem(row);
		ui->treeWidgetSorting->insertTopLevelItem(row - 1, item);
		ui->treeWidgetSorting->setCurrentItem(item);
		last_select_sorting_index = row - 1;

		//rerun index number
		int total_order = ui->treeWidgetSorting->topLevelItemCount();
		for (int i = 0; i < total_order; ++i)
		{
			QTreeWidgetItem *_item = ui->treeWidgetSorting->topLevelItem(i);
			_item->setText(0, QString::number(i+1));

		}

		ButtonUpdatePackingOrderPressed();

	}

}
void MainUI::ButtonMoveDownPackingOrderPressed()
{
	QTreeWidgetItem* item = ui->treeWidgetSorting->currentItem();
	int row = ui->treeWidgetSorting->currentIndex().row();
	int total_order = ui->treeWidgetSorting->topLevelItemCount();

	if (row < total_order - 1)
	{
		ui->treeWidgetSorting->takeTopLevelItem(row);
		ui->treeWidgetSorting->insertTopLevelItem(row + 1, item);
		ui->treeWidgetSorting->setCurrentItem(item);
		last_select_sorting_index = row + 1;

		//rerun index number
		int total_order = ui->treeWidgetSorting->topLevelItemCount();
		for (int i = 0; i < total_order; ++i)
		{
			QTreeWidgetItem *_item = ui->treeWidgetSorting->topLevelItem(i);
			_item->setText(0, QString::number(i+1));

		}

		ButtonUpdatePackingOrderPressed();
	}
}
void MainUI::ButtonUpdatePackingOrderPressed()
{
	int total_order = ui->treeWidgetSorting->topLevelItemCount();
	for (int i = 0; i < total_order; i++)
	{
		QTreeWidgetItem *itemsort = ui->treeWidgetSorting->topLevelItem(i);

		int item_number = itemsort->text(4).toInt();
		int item_index = item_number - 1;

		ObjectTransformationData *itm = dataprocess->items[item_index];
		itm->packing_order = i;


		QTreeWidgetItem *item = ui->treeWidget->topLevelItem(item_index);
		item->setText(12, QString::number(i));

	}

}

