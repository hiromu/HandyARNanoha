#include <global.h>
#include <ARMetaseq/ARMetaseq.h>
#include <HandyAR/FingertipPoseEstimation.h>

#include "bunny.h"
#include "capture.h"

#define RECORD_FILENAME         "record.avi"
#define RECORD_FILENAME_OUTPUT  "output.avi"
#define SCREENSHOT_FILENAME	"screenshot.png"
#define CALIBRATION_FILENAME	"calibration.txt"
#define EARTH_TEXTURE_FILENAME  "earthTexture.jpg"

// Sequence files name
#define SEQ_NAME	"Model2/default_%05d.mqo"
// Number of sequence files
#define N_FRAME		105
// Number of reset point
#define E_FRAME		35

#ifdef PROCESS_320x240
#define FINGERTIP_COORDINATE_FILENAME   "fingertip_320x240.dat"
#else
#define FINGERTIP_COORDINATE_FILENAME   "fingertip_640x480.dat"
#endif
char gszFingertipFilename[255] = FINGERTIP_COORDINATE_FILENAME;

CvFont font;
Capture gCapture;
FingertipPoseEstimation gFingertipPoseEstimation;

IplImage *cameraCenter;
IplImage *image;
IplImage *gray;
IplImage *handRegion;

bool fHandRegion = false;
bool fDistTrans = false;
bool fFingertipDetected = false;
bool fInputVideoFile = false;
char gszInputVideoFilename[255];
bool fFlipFrame = true;
bool fRecord = false;
bool fScreenshot = false;
CvVideoWriter *gpRecord = 0;
CvVideoWriter *gpRecordOutput = 0;

// Earth Texture
GLuint  gnEarthTexID;

// Magic Texture
int frame_num = 0;
MQO_SEQUENCE mqo_seq;

// Render Model
#define MODEL_BUNNY		1
#define MODEL_EARTH		2
#define MODEL_MAGIC		3
#define MODEL_COORDINATE_AXES	4
int gnModel = MODEL_COORDINATE_AXES;
