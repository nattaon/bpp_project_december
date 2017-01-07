#include "PointCloudBPPTextListIO.h"

PointCloudBPPTextListIO::PointCloudBPPTextListIO()
{

}


void PointCloudBPPTextListIO::WritePointCloudListForBPP(
	string filename, int total,
	int bin_x_dim, int bin_y_dim, int bin_z_dim,
	PointTypeXYZRGB bin_min_pos, PointTypeXYZRGB bin_max_pos,
	Eigen::Vector3f bin_major_vector, Eigen::Vector3f bin_middle_vector, Eigen::Vector3f bin_minor_vector,
	vector<string> array_pcd_filename,
	vector<int> items_x_dim, vector<int> items_y_dim, vector<int> items_z_dim,
	vector<PointTypeXYZRGB> items_min_pos, vector<PointTypeXYZRGB> items_max_pos,
	vector<Eigen::Vector3f> items_major_vector, vector<Eigen::Vector3f> items_middle_vector, vector<Eigen::Vector3f> items_minor_vector
	)
{

	cout << "WritePointCloudListForBPP" << endl;

	ofstream outfile;
	outfile.open(filename);

	outfile	<< bin_x_dim << " " << bin_y_dim << " " << bin_z_dim << endl;
	
	outfile << bin_min_pos.x << " " << bin_min_pos.y << " " << bin_min_pos.z << endl;
	outfile << bin_max_pos.x << " " << bin_max_pos.y << " " << bin_max_pos.z << endl;
	
	outfile << bin_major_vector[0] << " " << bin_major_vector[1] << " " << bin_major_vector[2] << endl;
	outfile << bin_middle_vector[0] << " " << bin_middle_vector[1] << " " << bin_middle_vector[2] << endl;
	outfile << bin_minor_vector[0] << " " << bin_minor_vector[1] << " " << bin_minor_vector[2] << endl;

	outfile << endl;
	outfile << total << endl;
	outfile << endl;

	for (int i = 0; i < total; i++)
	{
		outfile << array_pcd_filename[i] << endl;
		outfile << items_x_dim[i] << " " << items_y_dim[i] << " " << items_z_dim[i] << endl;
		
		outfile << items_min_pos[i].x << " " << items_min_pos[i].y << " " << items_min_pos[i].z << endl;
		outfile << items_max_pos[i].x << " " << items_max_pos[i].y << " " << items_max_pos[i].z << endl;
		
		outfile << items_major_vector[i][0] << " " << items_major_vector[i][1] << " " << items_major_vector[i][2] << endl;
		outfile << items_middle_vector[i][0] << " " << items_middle_vector[i][1] << " " << items_middle_vector[i][2] << endl;
		outfile << items_minor_vector[i][0] << " " << items_minor_vector[i][1] << " " << items_minor_vector[i][2] << endl;
		
		outfile << endl;

	}

	outfile.close();

}


//pass by reference(&) 
int PointCloudBPPTextListIO::ReadPointCloudListForBPP(string filename)
/*	, int &total)
int &bin_width, int &bin_height, int &bin_depth,
	vector<string> &array_pcd_filename,
	int *item_w, int *item_h, int *item_d)*/
{
	cout << "ReadListofPcdfromTextFile" << endl;

	ifstream infile;
	infile.open(filename);

	if (!infile.is_open())
	{
		cout << "cannot open file" << endl;
		return -1;
	}

	string sLine;
	size_t index1, index2;


	//get container dimension
	getline(infile, sLine); // read 1 line
	index1 = sLine.find(" ");
	index2 = sLine.find(" ",index1+1);

	//        index1  index2
	//             v   v  
	//index[i] =0123456789
	//string = "100 200 300"
	bin_x_dim = stoi(sLine.substr(0, index1));//i=0, length=3
	bin_y_dim = stoi(sLine.substr(index1+1, index2-index1));//i=4, length=3
	bin_z_dim = stoi(sLine.substr(index2+1));


	//get container min pos
	getline(infile, sLine); // read 1 line
	index1 = sLine.find(" ");
	index2 = sLine.find(" ", index1 + 1);

	bin_min_pos.x = stod(sLine.substr(0, index1));//index,length
	bin_min_pos.y = stod(sLine.substr(index1 + 1, index2 - index1));//index,length
	bin_min_pos.z = stod(sLine.substr(index2 + 1));

	//get container max pos
	getline(infile, sLine); // read 1 line
	index1 = sLine.find(" ");
	index2 = sLine.find(" ", index1 + 1);

	bin_max_pos.x = stod(sLine.substr(0, index1));//index,length
	bin_max_pos.y = stod(sLine.substr(index1 + 1, index2 - index1));//index,length
	bin_max_pos.z = stod(sLine.substr(index2 + 1));

	//get container major vector
	getline(infile, sLine); // read 1 line
	index1 = sLine.find(" ");
	index2 = sLine.find(" ", index1 + 1);

	bin_major_vector[0] = stod(sLine.substr(0, index1));//index,length
	bin_major_vector[1] = stod(sLine.substr(index1 + 1, index2 - index1));//index,length
	bin_major_vector[2] = stod(sLine.substr(index2 + 1));

	//get container middle vector
	getline(infile, sLine); // read 1 line
	index1 = sLine.find(" ");
	index2 = sLine.find(" ", index1 + 1);

	bin_middle_vector[0] = stod(sLine.substr(0, index1));//index,length
	bin_middle_vector[1] = stod(sLine.substr(index1 + 1, index2 - index1));//index,length
	bin_middle_vector[2] = stod(sLine.substr(index2 + 1));

	//get container minor vector
	getline(infile, sLine); // read 1 line
	index1 = sLine.find(" ");
	index2 = sLine.find(" ", index1 + 1);

	bin_minor_vector[0] = stod(sLine.substr(0, index1));//index,length
	bin_minor_vector[1] = stod(sLine.substr(index1 + 1, index2 - index1));//index,length
	bin_minor_vector[2] = stod(sLine.substr(index2 + 1));

	//blank enter
	getline(infile, sLine);

	//get items total
	getline(infile, sLine);
	total_items = stoi(sLine);

	//blank enter
	getline(infile, sLine);

	//get item data -> store in vector
	for (int i = 0; i < total_items; i++)
	{
		//get filename
		getline(infile, sLine);
		array_pcd_filename.push_back(string(sLine));//pcd file name

		//get dimension
		getline(infile, sLine);
		index1 = sLine.find(" ");
		index2 = sLine.find(" ", index1 + 1);
		items_x_dim.push_back(stoi(sLine.substr(0, index1)));//index, length
		items_y_dim.push_back(stoi(sLine.substr(index1 + 1, index2 - index1)));//index, length
		items_z_dim.push_back(stoi(sLine.substr(index2 + 1)));

		//get min pos
		getline(infile, sLine); // read 1 line
		index1 = sLine.find(" ");
		index2 = sLine.find(" ", index1 + 1);
		PointTypeXYZRGB item_min_pos;
		item_min_pos.x = stod(sLine.substr(0, index1));//index,length
		item_min_pos.y = stod(sLine.substr(index1 + 1, index2 - index1));//index,length
		item_min_pos.z = stod(sLine.substr(index2 + 1));
		items_min_pos.push_back(item_min_pos);

		//get max pos
		getline(infile, sLine); // read 1 line
		index1 = sLine.find(" ");
		index2 = sLine.find(" ", index1 + 1);
		PointTypeXYZRGB item_max_pos;
		item_max_pos.x = stod(sLine.substr(0, index1));//index,length
		item_max_pos.y = stod(sLine.substr(index1 + 1, index2 - index1));//index,length
		item_max_pos.z = stod(sLine.substr(index2 + 1));
		items_max_pos.push_back(item_max_pos);

		//get major vector
		getline(infile, sLine); // read 1 line
		index1 = sLine.find(" ");
		index2 = sLine.find(" ", index1 + 1);
		items_major_vector.push_back({ 
			stof(sLine.substr(0, index1)), 
			stof(sLine.substr(index1 + 1, index2 - index1)), 
			stof(sLine.substr(index2 + 1)) });

		//get middle vector
		getline(infile, sLine); // read 1 line
		index1 = sLine.find(" ");
		index2 = sLine.find(" ", index1 + 1);
		items_middle_vector.push_back({
			stof(sLine.substr(0, index1)),
			stof(sLine.substr(index1 + 1, index2 - index1)),
			stof(sLine.substr(index2 + 1)) });

		//get minor vector
		getline(infile, sLine); // read 1 line
		index1 = sLine.find(" ");
		index2 = sLine.find(" ", index1 + 1);
		items_minor_vector.push_back({
			stof(sLine.substr(0, index1)),
			stof(sLine.substr(index1 + 1, index2 - index1)),
			stof(sLine.substr(index2 + 1)) });

		//blank enter
		getline(infile, sLine);
	}


	return total_items;

}
