#include <HandyAR/ColorDistribution.h>

ColorDistribution::ColorDistribution(void)
{
	_nIndex = 0;
	_fReadyToCompute = false;
	_pMask = 0;
	_nWidth = 0;
	_nHeight = 0;
	_Threshold = 0;
}

ColorDistribution::~ColorDistribution(void)
{
	if(_pMask)
		cvReleaseImage(&_pMask);
}

void ColorDistribution::FeedFrame(IplImage *image)
{
	if(_fReadyToCompute)
		return;

	if(_nWidth == 0 && _nHeight == 0) {
		// Initialize
		_nWidth = image->width;
		_nHeight = image->height;
	}

	// Sampling
	for(int x = 0; x < _nWidth; x++) {
		for(int y = 0; y < _nHeight; y++) {
			CvScalar color = cvGet2D(image, y, x);
			for(int c = 1; c <= 2; c++)
				// Sample only Cr, Cb channels. (or G, R channels.)
				_Sample[x][y][c-1][_nIndex] = color.val[c];
		}
	}

	// Increase index
	_nIndex++;
	if(_nIndex == MAX_HISTORY) {
		_nIndex = 0;
		_fReadyToCompute = true;
	}

	// Compute mean and variance per-pixel
	if(_fReadyToCompute) {
		for(int x = 0; x < _nWidth; x++) {
			for(int y = 0; y < _nHeight; y++) {
				for(int c = 0; c < 2; c++) {
					// Mean
					_Mean[x][y][c] = 0;
					for(int h = 0; h < MAX_HISTORY; h++)
						_Mean[x][y][c] += _Sample[x][y][c][h];
					_Mean[x][y][c] /= ((float)MAX_HISTORY);

					// Variance
					_Var[x][y][c] = 0;
					for(int h = 0; h < MAX_HISTORY; h++)
						_Var[x][y][c] += (pow(_Sample[x][y][c][h] - _Mean[x][y][c], 2));
					_Var[x][y][c] /= ((float)MAX_HISTORY);
				}
			}
		}

		if(!_pMask)
			_pMask = cvCreateImage(cvSize(_nWidth, _nHeight), 8, 1);

		// threshold by variance
		for(int x = 0; x < _nWidth; x++)
			for(int y = 0; y < _nHeight; y++)
				cvSetReal2D(_pMask, y, x, 255 * (_Var[x][y][0]+_Var[x][y][1]) / 10000.0);
	}
}

IplImage *ColorDistribution::QueryMask()
{
	if(_fReadyToCompute == false)
		return 0;

	return _pMask;
}
