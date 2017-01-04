#ifndef MAINUI_H
#define MAINUI_H
#include "DataProcess.h"
#include "ViewerWindow.h"
#include "ViewerEmbeded.h"
#include "SharedHeader.h"

enum timermode { raw_input, bpp_animation };

namespace Ui {
class MainUI;
}

class MainUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainUI(QWidget *parent = 0);
    ~MainUI();
    void SetDataProcess(DataProcess* d);
    void SetViewerWindow(ViewerWindow* v);

private:
    Ui::MainUI *ui;
	DataProcess *dataprocess;
	ViewerWindow *viewerwindow;
	ViewerEmbeded *viewerembeded;

	timermode mode;

	bool isRegisterCameraCallback;

	QTime time;
	int timerId_kinect;
	int last_select_item_index;

	void timerEvent(QTimerEvent *event);

	bool isLoadPlaneParameter;

	bool eventFilter(QObject *object, QEvent *event);
	void keyPressEvent(QKeyEvent * event);
	void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void *stop_void);
	
	void ShowMinMaxCenterPoint(ObjectTransformationData *obj, string id_name);
	
	void SetCurrentCameraParameterToUI();
	void WritePlaneParamToUI();

	
private slots:
	//top menu
	///menu:viewer embedded
	void ButtonConnectPressed();
	void ButtonDisconnectPressed();
	void ButtonSavePointCloudFromViewerPressed();
	void ButtonLoadPointCloudToViewerPressed();

	///menu:parameter
	void ButtonLoadTransformParamPressed();
	void ButtonSaveTransformParamPressed();
	void ButtonLoadPassthroughParamPressed();
	void ButtonSavePassthroughParamPressed();

	//items above tab
	void RadioButtonAxisONSelected();
	void RadioButtonAxisOFFSelected();		
	void ButtonShowLoadedPointCloudPressed();
	void ButtonUndoLastedPointCloudPressed();
	void ButtonClearViewerPointCloudPressed();	
	void ButtonClearViewerShapePressed();

	//tab:camera
	void ButtonApplyParamtoCameraPressed();
	void ButtonResetCamParamPressed();

	void ButtonLoadCameraParamPressed();
	void ButtonSaveCameraParamPressed();

	void ButtonApplyCamRotationPressed();
	void ButtonApplyCamTranslationPressed();

	//tab:segmentation
	void ButtonApplyPlaneSegmentPressed();
	void ButtonCalculatePlaneTransformPressed();

	void ButtonRemovePlanePressed();
	void ButtonAlignPlaneToAxisCenterPressed();

	void ButtonLoadPlaneParamPressed();
	void ButtonSavePlaneParamPressed();

	//tab:filter+transform
	///Passthrough Filter
	void ButtonApplyCloudPassthroughPressed();
	void ButtonResetCloudPassthroughPressed();
	///Voxel Grid Filter
	void ButtonApplyVoxelGridPressed();
	///Outlier Removal
	void ButtonApplyOutlierPressed();
	///Transformation
	void ButtonApplyZeroCloudRotationPressed();
	void ButtonApplyZeroCloudTranslationPressed();
	void ButtonApplyCloudRotationPressed();
	void ButtonApplyCloudTranslationPressed();
	void ButtonResetCloudTransformationPressed();	
	///Cluster Extraction
	void ButtonShowClusterPressed();
	void ButtonExtractClusterPressed();
	void ButtonShowClusterBBPressed();
	void ButtonShowClusterVectorPressed();
	///Set Cloud Position
	void ButtonSetCloudCornerPressed();
	void ButtonSetCloudCenterPressed();
	void ButtonSetCloudAlignCornerPressed();
	void ButtonCalculateCloudTransformPressed();

	//BinPacking
	void ButtonCalculateBinPackingPressed();
	void ButtonTrackItemPositionPressed();

	void ButtonShowPackingTargetPressed();
	void ButtonShowPackingIndicatePressed();
	void ButtonShowPackingAnimationPressed();
	
	void ButtonShowZeroPackingPressed();
	void ButtonShowPrevPackingPressed();
	void ButtonShowNextPackingPressed();

	void ButtonSaveBinPackingInfoPressed();
	void ButtonLoadBinPackingInfoPressed();

	//cluster list
	void PressedTreeItem(QTreeWidgetItem *current_select_item);
	void ButtonLoadPointCloudToListPressed();
	void ButtonSavePointCloudFromListPressed();

	void ButtonAlignAllItemAxisPressed();
	void ButtonSaveAllItemPcdPressed();

	void ButtonLoadAllItemPressed();
	void ButtonSaveAllItemPressed();

	void ButtonRemoveItemPressed();
	void ButtonClearAllItemPressed();

};

#endif // MAINWINDOW_H
