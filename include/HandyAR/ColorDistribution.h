#ifndef _COLOR_DISTRIBUTION_H_
#define _COLOR_DISTRIBUTION_H_

#include <global.h>

#define DYNAMIC_PIXELS      true
#define STATIONARY_PIXELS   false

#define MAX_WIDTH   640
#define MAX_HEIGHT  480
#define MAX_HISTORY 30
#define MAX_CHANNEL 2

class ColorDistribution
{
public:
	ColorDistribution(void);
	~ColorDistribution(void);

	void FeedFrame(IplImage *image);
	IplImage *QueryMask();

private:
	// Statistics
	int  _nIndex;
	bool _fReadyToCompute;
	float _Sample[MAX_WIDTH][MAX_HEIGHT][MAX_CHANNEL][MAX_HISTORY];
	float _Mean[MAX_WIDTH][MAX_HEIGHT][MAX_CHANNEL];
	float _Var[MAX_WIDTH][MAX_HEIGHT][MAX_CHANNEL];

	// Result Mask Image
	IplImage *_pMask;

	// Information
	int  _nWidth;
	int  _nHeight;
	float _Threshold;
};

#endif // _COLOR_DISTRIBUTION_H_
