#ifndef TEXTFILEIO_H
#define TEXTFILEIO_H
#include "SharedHeader.h"
#include "PlaneSegmentParameterIO.h"
#include "CameraParameterIO.h"

class TextFileIO
{
public:
    TextFileIO();

	PlaneSegmentParameterIO *planeparam;
	CameraParameterIO *cameraparam;

};

#endif // TEXTFILEIO_H
