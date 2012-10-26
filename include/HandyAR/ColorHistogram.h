#ifndef _COLOR_HISTOGRAM_H_
#define _COLOR_HISTOGRAM_H_

#include <global.h>

#define NUM_HISTOGRAM_BINS  16
#define NUM_HISTOGRAM_HISTORY   5

class ColorHistogram
{
public:
	ColorHistogram(void);
	~ColorHistogram(void);

	void Reset();
	void FeedFrame(IplImage *srcImage, IplImage *maskImage);

	float QueryProbability(int value1, int value2, int value3);

private:
	// Histogram History
	int _nHist;      // Number of history
	int _nHistIndex; // Index of history
	int _nHistPixel[NUM_HISTOGRAM_HISTORY][NUM_HISTOGRAM_BINS][NUM_HISTOGRAM_BINS][NUM_HISTOGRAM_BINS];
	int _nNumPixel[NUM_HISTOGRAM_HISTORY];
	int _nTotalPixel;
};

#endif  // _COLOR_HISTOGRAM_H_
