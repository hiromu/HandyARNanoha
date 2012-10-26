#include "capture.h"

Capture::Capture(void)
{
	_fInitialized = false;
	_pCaptureOpenCV = 0;
	_pFrame = 0;
	_fFlipVertical = false;
}

Capture::~Capture(void)
{
	Terminate();
}

bool Capture::Initialize(bool flip, int index, char *filename)
{
	if (_fInitialized)
		goto Finished;

	_fFlipVertical = flip;

	// Try capture from file
	if (filename) {
		_pCaptureOpenCV = cvCreateFileCapture(filename);
		if (_pCaptureOpenCV)
			goto Initialized;
	}

	// Try capture from camera using OpenCV
	_pCaptureOpenCV = cvCreateCameraCapture(index);
	cvSetCaptureProperty(_pCaptureOpenCV, CV_CAP_PROP_FRAME_WIDTH, CAPTURE_SIZE_WIDTH);
	cvSetCaptureProperty(_pCaptureOpenCV, CV_CAP_PROP_FRAME_HEIGHT, CAPTURE_SIZE_HEIGHT);
	if (_pCaptureOpenCV)
		goto Initialized;

	// No more attempt, failure
	_fInitialized = false;
	goto Finished;

Initialized:
	// If successfully initialized, try first capture
	_fInitialized = CaptureFrame();

Finished:
	return _fInitialized;
}

void Capture::Terminate()
{
	if (_pCaptureOpenCV) {
		cvReleaseCapture(&_pCaptureOpenCV);
		_pCaptureOpenCV = 0;
	}

	_fInitialized = false;
}

bool Capture::CaptureFrame()
{
	if (_pCaptureOpenCV) {
		_pFrame = cvQueryFrame(_pCaptureOpenCV);
	} else {
		return false;
	}

	_nTickCount = cvGetTickCount();

	if (_pFrame == 0)
		return false;

	if (_fFlipVertical)
		cvFlip(_pFrame, _pFrame, 0);

	return true;
}

IplImage * Capture::QueryFrame()
{
	return _pFrame;
}

int64 Capture::QueryTickCount()
{
	return _nTickCount;
}
