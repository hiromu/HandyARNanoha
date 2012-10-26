#ifndef _CAPTURE_H_
#define _CAPTURE_H_

#include <opencv2/opencv.hpp>
#include <global.h>

#ifdef POINTGREY_CAPTURE
#include "FlyCap.h"
#endif

#define CAPTURE_OPENCV      0
#define CAPTURE_POINTGREY   1

#define CAPTURE_SIZE_WIDTH  640
#define CAPTURE_SIZE_HEIGHT 480

class Capture
{
public:
    Capture(void);
    ~Capture(void);

    bool Initialize(bool flip = false, int index = -1, char *filename = 0);
    void Terminate();

    bool        CaptureFrame();
    int64       QueryTickCount();
    IplImage *  QueryFrame();

private:
    bool        _fFlipVertical;
    bool        _fInitialized;
    int64       _nTickCount;
    IplImage *  _pFrame;
    CvCapture * _pCaptureOpenCV;
};

#endif // _CAPTURE_H_
