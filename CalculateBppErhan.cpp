#include "CalculateBppErhan.h"

int binpacking(int total_box_to_pack,
	int bin_w, int bin_h, int bin_d,
	int *box_w, int *box_h, int *box_d,
	int *_box_x, int *_box_y, int *_box_z,
	int *_orien_x, int *_orien_y, int *_orien_z,
	int *_bin_no, int *_item_no);

CalculateBppErhan::CalculateBppErhan(	
	int total_box_to_pack,
	int bin_w, int bin_h, int bin_d,
	int *box_w, int *box_h, int *box_d,
	int *box_x, int *box_y, int *box_z,
	int *orien_x, int *orien_y, int *orien_z,
	int *box_r, int *box_g, int *box_b,
	int *bin_no, int *item_no, int *item_order,
	int *item_rotation)
{
	bpp_total_box_to_pack = total_box_to_pack;

	bpp_bin_w = bin_w;
	bpp_bin_h = bin_h;
	bpp_bin_d = bin_d;

	bpp_box_w = box_w;
	bpp_box_h = box_h;
	bpp_box_d = box_d;

	bpp_box_x = box_x;
	bpp_box_y = box_y;
	bpp_box_z = box_z;

	bpp_orien_x = orien_x;
	bpp_orien_y = orien_y;
	bpp_orien_z = orien_z;

	bpp_box_r = box_r;
	bpp_box_g = box_g;
	bpp_box_b = box_b;

	bpp_bin_no = bin_no;
	bpp_item_no = item_no;
	bpp_item_order = item_order;

	bpp_item_rotation = item_rotation;

}


CalculateBppErhan::~CalculateBppErhan()
{
}

void CalculateBppErhan::CalculateBinpack()
{
	QTime timer;
	timer.start();
	binpacking(bpp_total_box_to_pack,
		bpp_bin_w, bpp_bin_h, bpp_bin_d,
		bpp_box_w, bpp_box_h, bpp_box_d,
		bpp_box_x, bpp_box_y, bpp_box_z,
		bpp_orien_x, bpp_orien_y, bpp_orien_z,
		bpp_bin_no, bpp_item_no);
	int nMilliseconds = timer.elapsed();
	cout << ":: binpacking :: timer elapsed " << nMilliseconds << " msec" << endl;

	/*binpacking(total_box_to_pack,
		bin_w, bin_h, bin_d,
		box_w, box_h, box_d,
		box_x, box_y, box_z,
		orien_x, orien_y, orien_z,
		bin_no, item_no);*/

	//cout << endl << "after binpacking" << endl;

	//printboxes_array();

	//cout << endl << "SortBoxesOrder();" << endl;

	//SortBoxesOrder();


	//InvertBoxesOrder();

	//printboxes_array();
}
void CalculateBppErhan::CalculateRotationCase()
{
	for (int i = 0; i < bpp_total_box_to_pack; i++)
	{
		if (bpp_bin_no[i] != 1) // not pack in same bin no.1
		{
			bpp_item_rotation[i] = -1;
		}
		else if (bpp_box_w[i] == bpp_orien_x[i] &&
			bpp_box_h[i] == bpp_orien_y[i] &&
			bpp_box_d[i] == bpp_orien_z[i])
		{
			bpp_item_rotation[i] = 0;
		}
		else if (bpp_box_w[i] == bpp_orien_x[i] &&
			bpp_box_h[i] == bpp_orien_z[i] &&
			bpp_box_d[i] == bpp_orien_y[i])
		{
			bpp_item_rotation[i] = 1;
		}
		else if (bpp_box_w[i] == bpp_orien_y[i] &&
			bpp_box_h[i] == bpp_orien_x[i] &&
			bpp_box_d[i] == bpp_orien_z[i])
		{
			bpp_item_rotation[i] = 2;
		}
		else if (bpp_box_w[i] == bpp_orien_y[i] &&
			bpp_box_h[i] == bpp_orien_z[i] &&
			bpp_box_d[i] == bpp_orien_x[i])
		{
			bpp_item_rotation[i] = 3;
		}
		else if (bpp_box_w[i] == bpp_orien_z[i] &&
			bpp_box_h[i] == bpp_orien_y[i] &&
			bpp_box_d[i] == bpp_orien_x[i])
		{
			bpp_item_rotation[i] = 4;
		}
		else if (bpp_box_w[i] == bpp_orien_z[i] &&
			bpp_box_h[i] == bpp_orien_x[i] &&
			bpp_box_d[i] == bpp_orien_y[i])
		{
			bpp_item_rotation[i] = 5;
		}
	}
}


void CalculateBppErhan::InvertBoxesOrder()
{
	int j = bpp_total_box_to_pack - 1;
	for (int i = 0; i < bpp_total_box_to_pack; i++,j--)
	{
		if (i < j)
		{
			swap_xyzwhd(i, j);
		}
		
	}
}
void CalculateBppErhan::SortBoxesOrder()
{
	

	//1. sort by y position(height)
	//cout << "sort y: 0 to " << bpp_total_box_to_pack << endl;
	SortMintoMax(bpp_box_y, 0, bpp_total_box_to_pack - 1);
	//printboxes_array();

	//find each same z range
	int z_start_index = 0;
	int y_start_index = 0;
	int x_start_index = 0;

	//2. loop in y axis(height)
	//cout << "y_start_index=" << y_start_index << endl;
	for (int i = y_start_index; i<bpp_total_box_to_pack; i++)
	{
		
		//cout << "i=" << i << endl;
		//3. if next y item is different value
		// sort z(deep) inside each group of y
		if (bpp_box_y[i] != bpp_box_y[i + 1])
		{
			//cout << "i " << i << "!=" << i + 1 << endl;
			//cout << "sort z: " << y_start_index << " to " << i << endl;
			SortMintoMax(bpp_box_z, y_start_index, i);
			//printboxes_array();
			
			//4. in each group of same z(deep)
			// sort x(left to right)
			z_start_index = y_start_index;
			//cout << " z_start_index=" << z_start_index << endl;
			for (int j = z_start_index; j<i + 1; j++)
			{			
				
				//cout << " j=" << j << endl;
				if (bpp_box_z[j] != bpp_box_z[j + 1])
				{
					//cout << "j " << j << "!=" << j + 1 << endl;
					//cout << "sort x: " << z_start_index << " to " << j << endl;
					SortMintoMax(bpp_box_x, z_start_index, j);
					//printboxes_array();

					z_start_index = j + 1;
				}
			}
			y_start_index = i + 1;
		}
		
	}

	//remember item order
	for (int i = 0; i < bpp_total_box_to_pack; i++)
	{
		if (bpp_bin_no[i]==1)
		{
			bpp_item_order[i] = i;
		}
		else
		{
			bpp_item_order[i] = -1;
		}
	}

	//cout << "bpp_item_order[i] = i;" << endl;
	//printboxes_array(); 
	//cout << endl;

	//arrage array again by item order
	SortMintoMax(bpp_item_order, 0, bpp_total_box_to_pack - 1);
	//cout << "quickSort(bpp_item_order, 0, bpp_total_box_to_pack - 1);" << endl;
	//printboxes_array();
	//cout << endl;

	//now array is arrange by order, so rewrite order (rerun number from 0->max , no space)
	int order_index = 0;
	for (int i = 0; i < bpp_total_box_to_pack; i++)
	{
		if (bpp_bin_no[i] == 1)
		{
			bpp_item_order[i] = order_index;
			order_index++;
		}
	}
	//cout << "after rerun order number" << endl;
	//printboxes_array();
	//cout << endl;

}

void CalculateBppErhan::quickSort(int *arr, int left, int right) //int *arr = int arr[]
{
	//sort min to max
	int i = left, j = right;
	int pivot = arr[(left + right) / 2];

	/* partition */
	while (i <= j) {
		while (arr[i] < pivot)
			i++;
		while (arr[j] > pivot)
			j--;

		if (i <= j) {

			swap_xyzwhd(i, j);
			//tmp = arr[i];
			//arr[i] = arr[j];
			//arr[j] = tmp;
			i++;
			j--;
		}
	};

	/* recursion */
	if (left < j)
		quickSort(arr, left, j);
	if (i < right)
		quickSort(arr, i, right);

}
void CalculateBppErhan::SortMaxtoMin(int *arr, int left, int right)
{
	//cout << "\nSortMaxtoMin" << endl;
	//PrintArrayValue();
	if (left == right) return;

	int i = left, j = right;
	int pivot = arr[(left + right) / 2];

	/* partition */
	while (i <= j) {
		while (arr[i] > pivot)
			i++;
		while (arr[j] < pivot)
			j--;

		if (i <= j)
		{
			swap_xyzwhd(i, j);
			i++;
			j--;
		}
	};

	/* recursion */
	if (left < j)
		SortMaxtoMin(arr, left, j);
	if (i < right)
		SortMaxtoMin(arr, i, right);
}
void CalculateBppErhan::SortMintoMax(int *arr, int left, int right)
{
	//cout << "\nSortMintoMax" << endl;
	//PrintArrayValue();
	int i = left, j = right;
	int pivot = arr[(left + right) / 2];

	/* partition */
	while (i <= j) {
		while (arr[i] < pivot)
			i++;
		while (arr[j] > pivot)
			j--;

		if (i <= j)
		{
			swap_xyzwhd(i, j);
			i++;
			j--;
		}
	};

	/* recursion */
	if (left < j)
		SortMintoMax(arr, left, j);
	if (i < right)
		SortMintoMax(arr, i, right);
}

void CalculateBppErhan::swap_xyzwhd(int i, int j)
{
	swap_pare(&bpp_box_x[i], &bpp_box_x[j]);
	swap_pare(&bpp_box_y[i], &bpp_box_y[j]);
	swap_pare(&bpp_box_z[i], &bpp_box_z[j]);

	swap_pare(&bpp_box_w[i], &bpp_box_w[j]);
	swap_pare(&bpp_box_h[i], &bpp_box_h[j]);
	swap_pare(&bpp_box_d[i], &bpp_box_d[j]);

	swap_pare(&bpp_orien_x[i], &bpp_orien_x[j]);
	swap_pare(&bpp_orien_y[i], &bpp_orien_y[j]);
	swap_pare(&bpp_orien_z[i], &bpp_orien_z[j]);

	swap_pare(&bpp_box_r[i], &bpp_box_r[j]);
	swap_pare(&bpp_box_g[i], &bpp_box_g[j]);
	swap_pare(&bpp_box_b[i], &bpp_box_b[j]);

	swap_pare(&bpp_bin_no[i], &bpp_bin_no[j]);
	swap_pare(&bpp_item_no[i], &bpp_item_no[j]);
	swap_pare(&bpp_item_order[i], &bpp_item_order[j]);

	swap_pare(&bpp_item_rotation[i], &bpp_item_rotation[j]);
}
void CalculateBppErhan::swap_pare(int *a, int *b)
{
	int tmp;
	tmp = *a;
	*a = *b;
	*b = tmp;
}



void CalculateBppErhan::printboxes_array()
{
	cout << "print box array" << endl;
	for (int i = 0; i < bpp_total_box_to_pack; i++) {
		cout
			<< "No." << bpp_item_no[i] << ", " 
			<< "bin_num:" << bpp_bin_no[i] << ", " 
			<< "whd:" << bpp_box_w[i] << "x" << bpp_box_h[i] << "x" << bpp_box_d[i] << ", " 
			<< "pos:" << bpp_box_x[i] << "," << bpp_box_y[i] << "," << bpp_box_z[i] << ", " 
			<< "orient:" << bpp_orien_x[i] << "," << bpp_orien_y[i] << "," << bpp_orien_z[i] << ", " 
			<< "order:" << bpp_item_order[i] 
			<< endl;
	}

}

bool CalculateBppErhan::CheckAllCanFit()
{
	int boxes_volumn = 0;
	int bin_volumn = bpp_bin_w *bpp_bin_h *bpp_bin_d;
	for (int i = 0; i < bpp_total_box_to_pack; i++)
	{
		boxes_volumn += (bpp_box_w[i] * bpp_box_h[i] * bpp_box_d[i]);
	}

	std::cout << "boxes:" << boxes_volumn << std::endl;
	std::cout << "bin:" << bin_volumn << std::endl;

	if (boxes_volumn > bin_volumn) return false;
	else return true;
}

