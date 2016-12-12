#ifndef MAINUI_H
#define MAINUI_H
#include "DataProcess.h"
#include "ViewerWindow.h"
#include "SharedHeader.h"

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

	QTime time;
	int timerId_kinect;

	void timerEvent(QTimerEvent *event);

	bool isLoadPlaneParameter;

	

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
	void ButtonLoadCameraParamPressed();
	void ButtonSaveCameraParamPressed();
	void ButtonLoadPlaneParamPressed();
	void ButtonSavePlaneParamPressed();
	
	//items above tab
	void RadioButtonAxisONSelected();
	void RadioButtonAxisOFFSelected();
	void RadioButtonBBONSelected();
	void RadioButtonBBOFFSelected();	
	void RadioButtonBBAABBSelected();
	void RadioButtonBBOBBSelected();		
	void ButtonShowLoadedPointCloudPressed();
	void ButtonClearViewerPointCloudPressed();	
	void ButtonClearViewerShapePressed();
	void ButtonUndoLastedPointCloudPressed();

	//tab:viewer parameter
	void ButtonApplyParamtoCameraPressed();
	void ButtonResetCamParamPressed();
	void ButtonApplyCamRotationPressed();
	void ButtonApplyCamTranslationPressed();

	//tab:cloud transformation
	void ButtonResetCloudTransformationPressed();	
	void ButtonApplyCloudRotationPressed();
	void ButtonApplyCloudTranslationPressed();
	void ButtonResetCloudPassthroughPressed();
	void ButtonApplyCloudPassthroughPressed();
	void ButtonSetCloudCenterPressed();
	void ButtonSetCloudCornerPressed();

	//tab:segmentation
	void ButtonApplyVoxelGridPressed();
	void ButtonApplyPlaneSegmentPressed();
	void ButtonGetPlaneTransformPressed();
	void ButtonRemovePlanePressed();
	void ButtonAlignPlaneToAxisCenterPressed();
	void ButtonApplyOutlierPressed();
	void ButtonShowClusterPressed();
	void ButtonExtractClusterPressed();
	void ButtonShowClusterPressed();
	

	//cluster list
	void PressedTreeItem(QTreeWidgetItem *current_select_item);
	void ButtonLoadPointCloudToListPressed();
	void ButtonSavePointCloudFromListPressed();
	void ButtonLoadAllItemPressed();
	void ButtonSaveAllItemPressed();
	void ButtonRemoveItemPressed();
	void ButtonClearAllItemPressed();
	void ButtonBinPackingPressed();
	void ButtonShowPrevPackingPressed();
	void ButtonShowNextPackingPressed();
	void ButtonTrackItemPositionPressed();
	


};

#endif // MAINWINDOW_H
