#include<iostream>
using namespace std;
#include "SharedHeader.h"
class CalculateBppErhan
{
public:
	CalculateBppErhan(int total_box_to_pack,
		int bin_w, int bin_h, int bin_d,
		int *box_w, int *box_h, int *box_d,
		int *box_x, int *box_y, int *box_z,
		int *orien_x, int *orien_y, int *orien_z,
		int *box_r, int *box_g, int *box_b,
		int *bin_no, int *item_no, int *item_order, 
		int *item_rotation);
	~CalculateBppErhan();

	void CalculateBinpack(
);

	void CalculateRotationCase();
	void SortBoxesOrder();
	void InvertBoxesOrder();

	bool CheckAllCanFit();

	void quickSort(int *arr, int left, int right);
	void SortMaxtoMin(int *arr, int left, int right);
	void SortMintoMax(int *arr, int left, int right);

	void swap_xyzwhd(int i, int j);
	void swap_pare(int *a, int *b);
	void printboxes_array();

	int bpp_total_box_to_pack;
	int bpp_bin_w, bpp_bin_h, bpp_bin_d;
	int	*bpp_box_w, *bpp_box_h, *bpp_box_d;
	int	*bpp_box_x, *bpp_box_y, *bpp_box_z;
	int	*bpp_orien_x, *bpp_orien_y, *bpp_orien_z;
	int	*bpp_box_r, *bpp_box_g, *bpp_box_b;
	int	*bpp_bin_no, *bpp_item_no, *bpp_item_order;
	int *bpp_item_rotation;

};

