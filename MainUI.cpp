#include "MainUI.h"

#define POINTCLOUD_DIR "../pcd_files"
#define LISTCOLUMN 6


MainUI::MainUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainUI)
{
	cout << "MainUI::MainUI(QWidget *parent)" << endl;

    ui->setupUi(this);
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
	connect(ui->bt_apply_removeplane, SIGNAL(clicked()), this, SLOT(ButtonRemovePlanePressed()));
	connect(ui->bt_setplane_align_axis, SIGNAL(clicked()), this, SLOT(ButtonAlignPlaneToAxisCenterPressed()));
	connect(ui->bt_apply_outlierremove, SIGNAL(clicked()), this, SLOT(ButtonApplyOutlierPressed()));
	connect(ui->bt_show_cluster, SIGNAL(clicked()), this, SLOT(ButtonShowClusterPressed()));
	connect(ui->bt_extract_cluster, SIGNAL(clicked()), this, SLOT(ButtonExtractClusterPressed()));

	//cluster list
	connect(ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(PressedTreeItem(QTreeWidgetItem *)));
	connect(ui->bt_item_load, SIGNAL(clicked()), this, SLOT(ButtonLoadPointCloudToListPressed()));
	connect(ui->bt_item_save, SIGNAL(clicked()), this, SLOT(ButtonSavePointCloudFromListPressed()));
	connect(ui->bt_all_load, SIGNAL(clicked()), this, SLOT(ButtonLoadAllItemPressed()));
	connect(ui->bt_all_save, SIGNAL(clicked()), this, SLOT(ButtonSaveAllItemPressed()));
	connect(ui->bt_item_remove, SIGNAL(clicked()), this, SLOT(ButtonRemoveItemPressed()));
	connect(ui->bt_item_clearall, SIGNAL(clicked()), this, SLOT(ButtonClearAllItemPressed()));
	connect(ui->bt_binpacking, SIGNAL(clicked()), this, SLOT(ButtonBinPackingPressed()));
	connect(ui->bt_order_previous, SIGNAL(clicked()), this, SLOT(ButtonShowPrevPackingPressed()));
	connect(ui->bt_order_next, SIGNAL(clicked()), this, SLOT(ButtonShowNextPackingPressed()));
	connect(ui->bt_track_item_pos, SIGNAL(clicked()), this, SLOT(ButtonTrackItemPositionPressed()));

	
	//resize tree column size	
	ui->treeWidget->header()->resizeSection(0, 40);
	ui->treeWidget->header()->resizeSection(5, 90);
	//myTreeWidget->headerView()->resizeSection(0 , 100);
}

MainUI::~MainUI()
{
    //delete ui;
	//delete program;
}

void MainUI::SetDataProcess(DataProcess* d) {dataprocess = d;}
void MainUI::SetViewerWindow(ViewerWindow* v) {viewerwindow = v;}


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
}
void MainUI::ButtonSaveCameraParamPressed()
{
	cout << "call ButtonSaveCameraParamPressed()" << endl;
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
	cout << "call RadioButtonAxisONSelected()" << endl;
}
void MainUI::RadioButtonAxisOFFSelected()
{
	cout << "call RadioButtonAxisOFFSelected()" << endl;
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
void MainUI::ButtonRemovePlanePressed()
{
	cout << "call ButtonRemovePlanePressed()" << endl;

	
	dataprocess->planeseg->RemovePlane(dataprocess->GetCurrentDisplayPointCloud());

	if (dataprocess->planeseg->isPlaneTransformDataAvailable())
	{
		viewerwindow->UpdateWindowCloudViewer(dataprocess->GetRemovedPlanePointCloud());
		dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetRemovedPlanePointCloud());
	}
	else
	{
		cout << "no planeparameter -> calculate plane param" << endl;
		dataprocess->planeseg->CalculatePlaneTransformation(dataprocess->GetOnlyPlanePointCloud());
		dataprocess->planeseg->RemovePlaneOutside(dataprocess->GetRemovedPlanePointCloud());

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

		viewerwindow->UpdateWindowCloudViewer(dataprocess->GetRemovedPlaneOutsidePointCloud());
		dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetRemovedPlaneOutsidePointCloud());
	}






}
void MainUI::ButtonAlignPlaneToAxisCenterPressed()
{
	cout << "call ButtonAlignPlaneToAxisCenterPressed()" << endl;
}
void MainUI::ButtonApplyOutlierPressed()
{
	cout << "call ButtonApplyOutlierPressed()" << endl;
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
	dataprocess->clusterextract->ShowClusterInColor(dataprocess->GetCurrentDisplayPointCloud());

	viewerwindow->UpdateWindowCloudViewer(dataprocess->GetColoredClusterPointCloud());
	//dataprocess->SetCurrentDisplayPointCloud(dataprocess->GetRemovedPlaneOutsidePointCloud());

}
void MainUI::ButtonExtractClusterPressed()
{
	cout << "call ButtonExtractClusterPressed()" << endl;

	dataprocess->SeparateContainerAndItems(dataprocess->clusterextract->GetExtractCluster());
	dataprocess->CalculateContainerTransformation();
	//cout << " check dataprocess->clusterextract->GetExtractCluster() size = " << dataprocess->clusterextract->GetExtractCluster().size() << endl;
 
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
}
	


