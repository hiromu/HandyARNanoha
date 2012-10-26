#include <HandyAR/ColorHistogram.h>

ColorHistogram::ColorHistogram(void)
{
	_nHist = 0;
	_nHistIndex = 0;
	_nTotalPixel = 0;
	for(int i = 0; i < NUM_HISTOGRAM_HISTORY; i++)
		_nNumPixel[i] = 0;
}

ColorHistogram::~ColorHistogram(void)
{
}

void ColorHistogram::Reset()
{
	_nHist = 0;
	_nHistIndex = 0;
	_nTotalPixel = 0;
	for(int i = 0; i < NUM_HISTOGRAM_HISTORY; i++)
		_nNumPixel[i] = 0;
}

void ColorHistogram::FeedFrame(IplImage *srcImage, IplImage *maskImage)
{
	if(!srcImage || !maskImage) {
		return;
	}

	// Remove the oldest history
	_nTotalPixel -= _nNumPixel[_nHistIndex];

	// Init current histogram history
	for(int i = 0; i < NUM_HISTOGRAM_BINS; i++)
		for(int j = 0; j < NUM_HISTOGRAM_BINS; j++)
			for(int k = 0; k < NUM_HISTOGRAM_BINS; k++)
				_nHistPixel[_nHistIndex][i][j][k] = 0;
	_nNumPixel[_nHistIndex] = 0;

	// Learn the Histogram
	for(int i = 0; i < maskImage->width; i++) {
		for(int j = 0; j < maskImage->height; j++) {
			if(cvGetReal2D(maskImage, j, i) == 255) {
				unsigned char value1 = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 0 ];
				unsigned char value2 = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 1 ];
				unsigned char value3 = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 2 ];

				_nHistPixel[_nHistIndex][value1 / NUM_HISTOGRAM_BINS][value2 / NUM_HISTOGRAM_BINS][value3 / NUM_HISTOGRAM_BINS]++;
				_nNumPixel[_nHistIndex]++;
			}
		}
	}

	// apply the new number of pixel
	_nTotalPixel += _nNumPixel[_nHistIndex];

	// Increase the index
	_nHistIndex++;
	if(_nHistIndex == NUM_HISTOGRAM_HISTORY)
		_nHistIndex = 0;

	// Increase the number of history
	if(_nHist < NUM_HISTOGRAM_HISTORY)
		_nHist++;
}

float ColorHistogram::QueryProbability(int value1, int value2, int value3)
{
	if(_nTotalPixel == 0) return 0;

	float numPixel = 0;
	float prob = 0;
	for(int i = 0; i < _nHist; i++)
		numPixel += _nHistPixel[i][value1 / NUM_HISTOGRAM_BINS][value2 / NUM_HISTOGRAM_BINS][value3 / NUM_HISTOGRAM_BINS];

	prob = numPixel / (float)_nTotalPixel;

	return prob;
}
