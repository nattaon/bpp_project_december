#include "BPPResultIO.h"

BPPResultIO::BPPResultIO()
{

}


void BPPResultIO::WriteBinPackingResult(
	string filename, int total,
	vector<int> packing_order, vector<int> item_index, vector<int> rotation_case,
	vector<PointTypeXYZRGB> target_position, vector<PointTypeXYZRGB> target_orientation)
{
	cout << "WriteBinPackingResult" << endl;

	ofstream outfile;
	outfile.open(filename);

	outfile << total << endl;

	outfile << endl;

	for (int i = 0; i < total; i++)
	{
		outfile << "packing_order " << packing_order[i] << endl;
		outfile << "ui_item_index " << item_index[i] << endl;
		outfile << "rotation_case " << rotation_case[i] << endl;

		outfile << target_position[i].x << " " << target_position[i].y << " " << target_position[i].z << endl;
		outfile << target_orientation[i].x << " " << target_orientation[i].y << " " << target_orientation[i].z << endl;

		outfile << endl;

	}

	outfile.close();
}

int BPPResultIO::ReadBinPackingResult(string filename)
{

	cout << "ReadBinPackingResult" << endl;

	ifstream infile;
	infile.open(filename);

	if (!infile.is_open())
	{
		cout << "cannot open file" << endl;
		return -1;
	}

	string sLine;
	size_t index1, index2;

	//get total_order
	getline(infile, sLine);
	total_order = stoi(sLine);

	//blank enter
	getline(infile, sLine);


	cout << "total_order=" << total_order << endl;
	for (int i = 0; i < total_order; i++)
	{

		//get packing_order  
		getline(infile, sLine);
		index1 = sLine.find(" ");
		packing_order.push_back(stoi(sLine.substr(index1+1)));

		//get ui_item_index 
		getline(infile, sLine);
		index1 = sLine.find(" ");
		item_index.push_back(stoi(sLine.substr(index1+1)));

		//get rotation_case   
		getline(infile, sLine);
		index1 = sLine.find(" ");
		rotation_case.push_back(stoi(sLine.substr(index1+1)));

		//get target_position
		getline(infile, sLine); // read 1 line
		index1 = sLine.find(" ");
		index2 = sLine.find(" ", index1 + 1);
		PointTypeXYZRGB item_target_position;
		item_target_position.x = stod(sLine.substr(0, index1));//index,length
		item_target_position.y = stod(sLine.substr(index1 + 1, index2 - index1));//index,length
		item_target_position.z = stod(sLine.substr(index2 + 1));
		target_position.push_back(item_target_position);

		//get target_orientation
		getline(infile, sLine); // read 1 line
		index1 = sLine.find(" ");
		index2 = sLine.find(" ", index1 + 1);
		PointTypeXYZRGB item_target_orientation;
		item_target_orientation.x = stoi(sLine.substr(0, index1));//index,length
		item_target_orientation.y = stoi(sLine.substr(index1 + 1, index2 - index1));//index,length
		item_target_orientation.z = stoi(sLine.substr(index2 + 1));
		target_orientation.push_back(item_target_orientation);
		
		//blank enter
		getline(infile, sLine);

	}



	return total_order;

}