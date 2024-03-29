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
	int current_display_packing_number;

	QTime time;
	int timerId_kinect;
	int last_select_item_index;
	int last_select_sorting_index;

	void timerEvent(QTimerEvent *event);

	bool isLoadPlaneParameter;


	//int *order_index;
	//int order_index_size;

	bool eventFilter(QObject *object, QEvent *event);
	void keyPressEvent(QKeyEvent * event);
	void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void *stop_void);
	
	void ShowMinMaxCenterPoint(ObjectTransformationData *obj, string id_name);
	
	void SetCurrentCameraParameterToUI();
	void WritePlaneParamToUI();
	void WriteContainerDataToUI(
		int bin_x_dim, int bin_y_dim, int bin_z_dim,
		PointTypeXYZRGB bin_min_pos, PointTypeXYZRGB bin_max_pos);
	void WriteItemDataToUI(int index,
		int box_x_dim, int box_y_dim, int box_z_dim, size_t cloud_size,
		PointTypeXYZRGB box_min_pos, PointTypeXYZRGB box_max_pos,
		string file_name);

	void ShowPackingCurrentOrder();

	void Call_LoadCameraParam(string filename);
	void Call_LoadAllItemsTextToUI(string filename);
	void Call_LoadBinPackingInfo(string filename);

	
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

	void ButtonTestProgrammePressed();
	void ButtonTestInput1Pressed();
	void ButtonTestInput2Pressed();
	void ButtonTestInput3Pressed();
	void ButtonTestInput4Pressed();

	//tab:camera
	void ButtonApplyParamtoCameraWindowPressed();
	void ButtonApplyParamtoCameraEmbededPressed();
	void ButtonResetCamParamPressed();

	void ButtonLoadCameraParamPressed();
	void ButtonSaveCameraParamPressed();

	void ButtonApplyCamRotationPressed();
	void ButtonApplyCamTranslationPressed();

	void ButtonGetRegisterCameraCallbackPressed();

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
	void ButtonSetCloudAlignAxisPressed();
	void ButtonCalculateCloudTransformPressed();
	void ButtonFillEmptyPressed();
	void ButtonFillInvertPressed();

	//cluster list
	void CheckboxShowEmbededAxisPressed();
	void CheckboxShowEmbededBoundingPressed();

	void PressedTreeItem(QTreeWidgetItem *current_select_item);
	void ButtonLoadPointCloudToListPressed();
	void ButtonSavePointCloudFromListPressed();

	void ButtonAlignAllItemAxisPressed();
	void ButtonSaveAllItemPointcloudToPcdPressed();

	void ButtonFillEmptyAllPressed();
	void ButtonFillInvertAllPressed();

	void ButtonLoadAllItemsTextToUIPressed();
	void ButtonSaveAllItemsUIToTextPressed();

	void ButtonRemoveItemPressed();
	void ButtonClearAllItemPressed();

	//BinPacking
	void ButtonCalculateBinPackingPressed();

	void ButtonShowPackingTargetPressed();
	void ButtonShowPackingIndicatePressed();
	void ButtonShowPackingAnimationPressed();

	void ButtonShowProjectionInputPositionPressed();
	void ButtonShowTestRectanglePositionPressed();
	void ButtonShowTestCubePressed();

	void ButtonSetContainerItemsYzeroPressed();
	void ButtonUpdateContainerItemsToDataprocessPressed();


	//Sorting
	void PressedTreeSorting(QTreeWidgetItem *current_select_item);

	void ButtonSaveBinPackingInfoPressed();
	void ButtonLoadBinPackingInfoPressed();

	void ButtonMoveUpPackingOrderPressed();
	void ButtonMoveDownPackingOrderPressed();
	void ButtonUpdatePackingOrderPressed();
	
	void ButtonShowZeroPackingPressed();
	void ButtonShowPrevPackingPressed();
	void ButtonShowNextPackingPressed();

	//pointsize
	void ButtonPointsizeWindowPressed();
	void ButtonPointsizeEmbededPressed();
	


};

#endif // MAINWINDOW_H
