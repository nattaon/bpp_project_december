#ifndef TEXTFILEIO_H
#define TEXTFILEIO_H
#include "SharedHeader.h"
#include "PlaneSegmentParameterIO.h"
#include "CameraParameterIO.h"
#include "PointCloudBPPTextListIO.h"

class TextFileIO
{
public:
    TextFileIO();

	PlaneSegmentParameterIO *planeparam;
	CameraParameterIO *cameraparam;
	PointCloudBPPTextListIO * pointcloudbpptext;
};

#endif // TEXTFILEIO_H
