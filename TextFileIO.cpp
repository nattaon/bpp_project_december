#include "TextFileIO.h"

TextFileIO::TextFileIO()
{
	cout << "TextFileIO()" << endl;

	planeparam = new PlaneSegmentParameterIO();
	cameraparam = new CameraParameterIO();
	pointcloudbpptext = new PointCloudBPPTextListIO();

	//if not new class, we can still call class, 
	//but declare variable in class in not be initialse
	//so we will unable to read memory of tat variable in class

}
