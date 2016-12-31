#include "PointCloudBPPTextListIO.h"

PointCloudBPPTextListIO::PointCloudBPPTextListIO()
{

}


void PointCloudBPPTextListIO::WritePointCloudListForBPP(string filename, int total,
	int bin_width, int bin_height, int bin_depth,
	vector<string> array_pcd_filename,
	int *item_w, int *item_h, int *item_d)
{

	cout << "WriteListofPcdtoTextFile" << endl;

	ofstream outfile;
	outfile.open(filename);

	outfile
		<< bin_width << " " << bin_height << " " << bin_depth << endl
		<< total << endl;

	for (int i = 0; i < total; i++)
	{
		outfile << array_pcd_filename[i] << endl;
		outfile << item_w[i] << " "
			<< item_h[i] << " "
			<< item_d[i] << endl;
	}

	outfile.close();

}


//pass by reference(&) 
void PointCloudBPPTextListIO::ReadPointCloudListForBPP(string filename, int &total,
	int &bin_width, int &bin_height, int &bin_depth,
	vector<string> &array_pcd_filename,
	int *item_w, int *item_h, int *item_d)
{
	cout << "ReadListofPcdfromTextFile" << endl;

	ifstream infile;
	infile.open(filename);

	if (!infile.is_open())
	{
		cout << "cannot open file" << endl;
		return;
	}

	string sLine;

	//1.get bin size
	getline(infile, sLine);
	size_t index1 = 0;
	size_t index2 = 0;

	index1 = sLine.find(" ");
	index2 = sLine.find_last_of(" ");

	bin_width = stod(sLine.substr(0, index1));
	bin_height = stod(sLine.substr(index1, index2));
	bin_depth = stod(sLine.substr(index2));

	//2.get items total
	getline(infile, sLine);
	total = stod(sLine);


	//item_w = new int[total];
	//item_h = new int[total];
	//item_d = new int[total];


	//3.get array of pcd filename and item dimension
	for (int i = 0; i < total; i++)
	{
		getline(infile, sLine);
		array_pcd_filename.push_back(string(sLine));//pcd file name

		getline(infile, sLine);
		index1 = sLine.find(" ");
		index2 = sLine.find_last_of(" ");

		Eigen::Vector3f dimension;

		item_w[i] = stod(sLine.substr(0, index1));
		item_h[i] = stod(sLine.substr(index1, index2));
		item_d[i] = stod(sLine.substr(index2));


	}

	//cout << "item_w " << item_w << endl;
	//cout << "item_h " << item_h << endl;
	//cout << "item_d " << item_d << endl;



}
