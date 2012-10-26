#include <HandyAR/FingerTip.h>

FingerTip::FingerTip(void)
{
	_pHandImage = 0;
	_pDistImage = 0;
	_pDistImage2 = 0;
	_numFingerPoint = 0;
	_circleTemplate5 = 0;
	_circleMatch5 = 0;
	_numKmeansPoint = 0;
	_nKmeansPointFront = 0;
	_nKmeansPointRear = 0;
	_nCluster = 5;

	for(int i = 0; i < MAX_FINGER_POINT; i++)
		_pKalman[i] = 0;
}

FingerTip::~FingerTip(void)
{
	if(_pHandImage)
		cvReleaseImage(&_pHandImage);
	if(_pDistImage)
		cvReleaseImage(&_pDistImage);
	if(_pDistImage2)
		cvReleaseImage(&_pDistImage2);
	for(int i = 0; i < MAX_FINGER_POINT; i++)
		if(_pKalman[i])
			cvReleaseKalman(&_pKalman[i]);
	if(_circleTemplate5)
		cvReleaseImage(&_circleTemplate5);
	if(_circleMatch5)
		cvReleaseImage(&_circleMatch5);
}

void FingerTip::FloodFill(IplImage *src, IplImage *dst, CvPoint point)
{
	int dir[8][2] = { {-1, 1 }, { 0, 1 }, { 1, 1 },
			  {-1, 0 },           { 1, 0 },
			  {-1,-1 }, { 0,-1 }, { 1,-1 } };

	cvSetReal2D(dst, point.y, point.x, cvGetReal2D(src, point.y, point.x));

	for(int i = 0; i < 8; i++) {
		CvPoint nextPoint = cvPoint(point.x + dir[i][0], point.y + dir[i][1]);
		if(nextPoint.x < 0 || nextPoint.x >= src->width || nextPoint.y < 0 || nextPoint.y >= src->height)
			continue;

		if(cvGetReal2D(src, nextPoint.y, nextPoint.x) >  0 && cvGetReal2D(dst, nextPoint.y, nextPoint.x) == 0)
			FloodFill(src, dst, nextPoint);
	}
}

int FingerTip::FindFingerTipCandidates(IplImage * handRegion)
{
	if(!_pDistImage) {
		_pDistImage = cvCreateImage(cvGetSize(handRegion), IPL_DEPTH_32F, 1);
		_pDistImage->origin = handRegion->origin;
	}
	if(!_pDistImage2) {
		_pDistImage2 = cvCreateImage(cvGetSize(handRegion), IPL_DEPTH_32F, 1);
		_pDistImage2->origin = handRegion->origin;
	}
	if(!_pHandImage) {
		_pHandImage = cvCreateImage(cvGetSize(handRegion), 8, 1);
		_pHandImage->origin = handRegion->origin;
	}

	// Get Distance Transform
	cvDistTransform(handRegion, _pDistImage2, CV_DIST_L2, CV_DIST_MASK_3, NULL, NULL);
	_maxDistValue = 0;
	_maxDistPoint = cvPoint(0, 0);
	cvMinMaxLoc(_pDistImage2, 0, &_maxDistValue, 0, &_maxDistPoint);

	// Find Contours to get a single connected component for the hand
	// that has the max distance point in it.
	CvMemStorage *pStorage = cvCreateMemStorage(0);
	CvSeq *pContours;   
	int nContours = cvFindContours(handRegion, pStorage, &pContours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	cvSetZero(_pHandImage);
	for(; pContours; pContours = pContours->h_next) {
		cvDrawContours(_pHandImage, pContours, cvScalar(255), cvScalar(0), 0, -1, 8);
		if(cvGetReal2D(_pHandImage, _maxDistPoint.y, _maxDistPoint.x) == 255) {
			cvSetZero(_pHandImage);
			cvDrawContours(_pHandImage, pContours, cvScalar(255), cvScalar(0), 0, -1, 8);
			cvSetZero(_pDistImage);
			cvCopy(_pDistImage2, _pDistImage, _pHandImage);
			break;
		}
	}
	cvClearMemStorage(pStorage);

	// Find Second Rank Points
	FindSecondRankPoints();
	if(_circleMaxDistIndex == -1)
		return 0;

	// Find End Points
	_numEndPoint = 0;

	for(int i = 0; i < _numCirclePoint; i++) {
		// check image boundary
		if(_circlePoint[i].x < _circleDist[i] * FINGERTIP_CONDITION_RANGE || _circlePoint[i].y < _circleDist[i] * FINGERTIP_CONDITION_RANGE || _pDistImage->width  - _circlePoint[i].x <= _circleDist[i] * FINGERTIP_CONDITION_RANGE || _pDistImage->height - _circlePoint[i].y <= _circleDist[i] * FINGERTIP_CONDITION_RANGE)
			continue;

		// check finger tip condition
		if(!CheckFingerTipCondition(_pHandImage, _circlePoint[i].x, _circlePoint[i].y, _circleDist[i] * FINGERTIP_CONDITION_RANGE))
			continue;

		// now this is a finger tip
		//AddFingerTip(i);
		_endPoint[_numEndPoint] = _circlePoint[i];
		_endPointDist[_numEndPoint] = _circleDist[i];
		_endPointScore[_numEndPoint] = GetFingerTipScore(_pDistImage, _circlePoint[i].x, _circlePoint[i].y);
		_numEndPoint++;
	}

	// Get only one fingertip among adjacent end points
	RemoveAdjacentEndPoints();

	// Sort points by the distance to the center
	//SortEndPoints();

	return _numEndPoint;
}

void FingerTip::SortEndPoints()
{
	for(int i = 0; i < _numEndPoint; i++) {
		for(int j = i + 1; j < _numEndPoint; j++) {
			if(_endPointNumAdj[i] < _endPointNumAdj[j]) {
				CvPoint tempPoint = _endPoint[i];
				_endPoint[i] = _endPoint[j];
				_endPoint[j] = tempPoint;

				float tempDist = _endPointDist[i];
				_endPointDist[i] = _endPointDist[j];
				_endPointDist[j] = tempDist;

				float tempScore = _endPointScore[i];
				_endPointScore[i] = _endPointScore[j];
				_endPointScore[j] = tempScore;

				int tempNumAdj = _endPointNumAdj[i];
				_endPointNumAdj[i] = _endPointNumAdj[j];
				_endPointNumAdj[j] = tempNumAdj;
			}
		}
	}
}

void FingerTip::RemoveAdjacentEndPoints()
{
	// Check if there is already a fingertip that is adjacent to this one
	for(int i = 0; i < _numEndPoint; i++) {
		_endPointNumAdj[i] = 0;
		for(int j = i + 1; j < _numEndPoint; j++) {
			float tempDist = sqrt(pow((float)_endPoint[i].x-_endPoint[j].x, 2) + pow((float)_endPoint[i].y-_endPoint[j].y, 2));
			tempDist -= _endPointDist[i];
			if(_endPointDist[j] > tempDist) {
				// exist, remove one that is close to the center
				float distToMaxI = sqrt(pow((float)_endPoint[i].x-_maxDistPoint.x, 2) + pow((float)_endPoint[i].y-_maxDistPoint.y, 2));
				float distToMaxJ = sqrt(pow((float)_endPoint[j].x-_maxDistPoint.x, 2) + pow((float)_endPoint[j].y-_maxDistPoint.y, 2));
				if(distToMaxI < distToMaxJ) {
					// Take endPoint[j]
					_endPoint[i] = _endPoint[j];
					_endPointDist[i] = _endPointDist[j];
					_endPointScore[i] = _endPointScore[j];
					_endPointNumAdj[i]++;
				}

				// Remove old endPoint[j]
				_endPoint[j] = _endPoint[_numEndPoint-1];
				_endPointDist[j] = _endPointDist[_numEndPoint-1];
				_endPointScore[j] = _endPointScore[_numEndPoint-1];
				_numEndPoint--;
				j--;
			}
		}
	}
}

void FingerTip::AddFingerTip(int index)
{
	// Check if there is already a fingertip that is adjacent to this one
	for(int i = 0; i < _numEndPoint; i++) {
		float tempDist = sqrt(pow((float)_circlePoint[i].x-_circlePoint[index].x, 2) + pow((float)_circlePoint[i].y-_circlePoint[index].y, 2));
		tempDist -= _circleDist[i];
		if(_circleDist[index] > tempDist) {
			// exist, don't add but update
			float distToMaxI = sqrt(pow((float)_circlePoint[i].x-_maxDistPoint.x, 2) + pow((float)_circlePoint[i].y-_maxDistPoint.y, 2));
			float distToMaxIndex = sqrt(pow((float)_circlePoint[index].x-_maxDistPoint.x, 2) + pow((float)_circlePoint[index].y-_maxDistPoint.y, 2));
			if(distToMaxI < distToMaxIndex) {
				// new point farther than the old one to the max point
				_endPoint[i] = _circlePoint[index];
				_endPointDist[i] = _circleDist[index];
				return;
			}
		}
	}

	// No previous point exist, thus add a new point
	_endPoint[_numEndPoint] = _circlePoint[index];
	_endPointDist[_numEndPoint] = _circleDist[index];
	_numEndPoint++;
}

void FingerTip::FindSecondRankPoints()
{
	_circleMaxDistValue = -1;
	_circleMaxDistIndex = -1;
	_numCirclePoint = 0;

	float Radius;
	int i1 = _maxDistPoint.y - 4 * _maxDistValue;
	int i2 = _maxDistPoint.y + 4 * _maxDistValue;
	int j1 = _maxDistPoint.x - 4 * _maxDistValue;
	int j2 = _maxDistPoint.x + 4 * _maxDistValue;
	if(i1 < 0)
		i1 = 0;
	if(i2 > _pDistImage->height)
		i2 = _pDistImage->height;
	if(j1 < 0)
		j1 = 0;
	if(j2 > _pDistImage->width)
		j2 = _pDistImage->width;

	for(int i = i1; i < i2; i++) {
		for(int j = j1; j < j2; j++) {
			if(cvGetReal2D(_pDistImage, i, j) == 0)
				continue;

			Radius = (float)cvGetReal2D(_pDistImage, i, j);

			if(GetRankInNeighbor(j, i) == 2) {
				//cvSetReal2D(_pDistImage, i, j, 255);
				if(Radius > 3 && Radius <= _maxDistValue * 0.3) {
					_circlePoint[_numCirclePoint] = cvPoint(j, i);
					_circleDist [_numCirclePoint] = Radius;
					if(_circleMaxDistValue < Radius) {
						_circleMaxDistValue = Radius;
						_circleMaxDistIndex = _numCirclePoint;
					}
					_numCirclePoint++;
				}
			}
		}
	}
}

int FingerTip::GetRankInNeighbor(int x, int y)
{
	double mySelf = cvGetReal2D(_pDistImage, y, x);
	int myRank = 0;
	for(int i = -1; i <= 1; i++) {
		for(int j = -1; j <= 1; j++) {
			if(i+x < 0 || i+x >= _pDistImage->width || j+y < 0 || j+y >= _pDistImage->height)
				continue;

			if(cvGetReal2D(_pDistImage, j+y, i+x) >= mySelf)
				myRank++;
		}
	}

	return myRank;
}

void FingerTip::FindEndPoints(int index, int prevIndex, int prevIndex2)
{
	// check image boundary
	if(!(_circlePoint[index].x < _circleDist[index] * FINGERTIP_CONDITION_RANGE || _circlePoint[index].y < _circleDist[index] * FINGERTIP_CONDITION_RANGE || _pDistImage->width  - _circlePoint[index].x < _circleDist[index] * FINGERTIP_CONDITION_RANGE ||_pDistImage->height - _circlePoint[index].y < _circleDist[index] * FINGERTIP_CONDITION_RANGE)) {
		// check fingertip condition
		if(CheckFingerTipCondition(_pDistImage, _circlePoint[index].x, _circlePoint[index].y, _circleDist [index] * FINGERTIP_CONDITION_RANGE)) {
			// this point is an endpoint
			_endPoint[_numEndPoint] = _circlePoint[index];
			_endPointDist[_numEndPoint] = _circleDist[index];
			_endPointScore[_numEndPoint] = GetFingerTipScore(_pDistImage, _circlePoint[index].x, _circlePoint[index].y);
			_numEndPoint++;
		}
	}

	// check this point visited
	_circleVisited[index] = true;

	// go to connected unvisited point
	for(int i = 0; i < _numCirclePoint; i++) {
		if(_circleVisited[i]) continue;
		float tempDist = sqrt(pow((float)_circlePoint[i].x-_circlePoint[index].x, 2) + pow((float)_circlePoint[i].y-_circlePoint[index].y, 2));
		if(_circleDist[index] + _circleDist[i] > tempDist)
			// we have a point to visit
			FindEndPoints(i, index, prevIndex);
	}

/*
	// get the closest unvisited point
	float minDist = 10000000;
	int minIndex = -1;
	bool fEndpoint = true;
	do {
		minDist = 10000000;
		minIndex = -1;
		for(int i = 0; i < _numCirclePoint; i++) {
			if(_circleVisited[i])
				continue;

			float tempDist = sqrt(pow((float)_circlePoint[i].x-_circlePoint[index].x, 2) + pow((float)_circlePoint[i].y-_circlePoint[index].y, 2));
			//tempDist -= _circleDist[i];
			if(_circleDist[index] + _circleDist[i] > tempDist && tempDist + _circleDist[i] < minDist) {
				minDist = tempDist + _circleDist[i];
				minIndex = i;
			}
		}
		if(minIndex >= 0) {
		// we have a point to visit
		FindEndPoints(minIndex, index, prevIndex);
		fEndpoint = false;
		}
	} while(minIndex >= 0);

	if(fEndpoint)
	{
		// check image boundary
		if(_circlePoint[index].x < _circleDist[index] * FINGERTIP_CONDITION_RANGE || _circlePoint[index].y < _circleDist[index] * FINGERTIP_CONDITION_RANGE || _pDistImage->width  - _circlePoint[index].x < _circleDist[index] * FINGERTIP_CONDITION_RANGE || _pDistImage->height - _circlePoint[index].y < _circleDist[index] * FINGERTIP_CONDITION_RANGE)
			return;

			// check the size
			float sizeRatio  = _circleDist[index] / _circleDist[prevIndex];
			float sizeRatio2 = _circleDist[index] / _circleDist[prevIndex2];
			if(sizeRatio < 0.9 || sizeRatio > 1.1)
				return;
			if(sizeRatio2 < 0.9 || sizeRatio2 > 1.1)
				return;

		// check fingertip condition
		if(!CheckFingerTipCondition(_pDistImage, _circlePoint[index].x, _circlePoint[index].y, _circleDist [index] * FINGERTIP_CONDITION_RANGE))
			return;

		// this point is an endpoint
		_endPoint[_numEndPoint] = _circlePoint[index];
		_endPointDist[_numEndPoint] = _circleDist[index];
		_endPointScore[_numEndPoint] = GetFingerTipScore(_pDistImage, _circlePoint[index].x, _circlePoint[index].y);
		_numEndPoint++;
	}
*/
}

void FingerTip::TrackFingerTips()
{
	// Track Previous FingerTips
	// Start New FingerTip Tracking for untracked candidates
}

int FingerTip::FindFingerTipCandidatesByTemplateMatching(IplImage * handRegion)
{
	if(!_pDistImage) {
		_pDistImage = cvCreateImage(cvGetSize(handRegion), IPL_DEPTH_32F, 1);
		_pDistImage->origin = handRegion->origin;
	}

	// Get Distance Transform
	//cvDistTransform(handRegion, _pDistImage, CV_DIST_L2, CV_DIST_MASK_3, NULL, NULL);
	//cvMinMaxLoc(_pDistImage, 0, &_maxDistValue, 0, &_maxDistPoint);

	// Make a Circle Template
	int nTemplateSize = 25;
	int nMidPos = 12;
	int nRadius = 8;

	if(!_circleTemplate5) {
		_circleTemplate5 = cvCreateImage(cvSize(nTemplateSize, nTemplateSize), 8, 1);
		_circleTemplate5->origin = handRegion->origin;
		cvCircle(_circleTemplate5, cvPoint(nMidPos, nMidPos), nRadius, CV_RGB(255, 255, 255), -1, 8, 0);
	}
	if(!_circleMatch5) {
		_circleMatch5 = cvCreateImage(cvSize(handRegion->width - _circleTemplate5->width + 1, handRegion->height - _circleTemplate5->height + 1), IPL_DEPTH_32F, 1);
		_circleMatch5->origin = handRegion->origin;
	}

	_numEndPoint = 0;

	// Template Matching with radius 5
	cvSetImageROI(handRegion, cvRect(0, 0, handRegion->width,handRegion->height));
	cvMatchTemplate(handRegion, _circleTemplate5, _circleMatch5, CV_TM_CCORR_NORMED);
	cvResetImageROI(handRegion);

	// Find End Points
	for(int i = 0; i < _circleMatch5->height; i++) {
		for(int j = 0; j < _circleMatch5->width; j++) {
			double match = cvGetReal2D(_circleMatch5, i, j);
/*
			if(cvGetReal2D(handRegion, i + 22, j + 22) > 0 && cvGetReal2D(fingerTip, i + 22, j + 22) < match && cvGetReal2D(dist, i + 22, j + 22) >= 5 && image->imageData[(i + 22) * image->widthStep + (j + 22) * 3 + 0] == 255 && CheckFingerTipCondition(handRegion, j + nMidPos, i + nMidPos, nRadius * FINGERTIP_CONDITION_RANGE) && match >= 0.7)
*/
			if(CheckFingerTipCondition(handRegion, j + nMidPos, i + nMidPos, nRadius * FINGERTIP_CONDITION_RANGE) && match >= 0.7)
			{
				//cvCircle(fingerTip, cvPoint(j + 22, i + 22), 15, cvScalar(match), -1, 8, 0);
				//cvCircle(image, cvPoint(j + 22, i + 22), 15, CV_RGB(255, 0, 0), -1, 8, 0);
				_endPoint[_numEndPoint] = cvPoint(j+nMidPos, i+nMidPos);
				_endPointDist[_numEndPoint] = nRadius;
				_numEndPoint++;
			}
		}
	}

	// Get only one fingertip among adjacent end points
	RemoveAdjacentEndPoints();

	return _numEndPoint;
}

bool FingerTip::CheckFingerTipCondition(IplImage *image, int x, int y, int size)
{
	int numHandRegion = 0;
	int i1, j1;
	for(int i = -1; i <= 1; i++) {
		for(int j = -1; j <= 1; j++) {
			if(i == 0 && j == 0)
				continue;
			if(i*size+x < 0 || i*size+x >= image->width || j*size+y < 0 || j*size+y >= image->height)
				continue;

			if(cvGetReal2D(image, j*size+y, i*size+x) > 2) // <<-- 0 or 1 or 2 or ??
			{
				numHandRegion++;
				if(numHandRegion > 2)
					return false;
				if(numHandRegion == 2)
					if(!(i==i1 && abs(j-j1)==1 || j==j1 && abs(i-i1)==1))
						return false;
				if(numHandRegion == 1) {
					i1 = i;
					j1 = j;
				}
			}

		}
	}
	if(numHandRegion == 0)
		return false;

	return true;
}

float FingerTip::GetFingerTipScore(IplImage *distImage, int x, int y)
{
	float r = cvGetReal2D(distImage, y, x);
	int R = FINGERTIP_CONDITION_RANGE * r;
	int di[4] = { 0, 1, 0, -1 };    // row step
	int dj[4] = { 1, 0, -1, 0 };    // col step
	int i = y - R;
	int j = x - R;
	int step = 2 * R;

	float dist = 0;
	float distMax = 0;
	int   numHandRegion = 0;
	int l = 0;
	for(int k = 0; k < 4; k++) {
		do {
			dist = cvGetReal2D(distImage, i, j);

			// count hand region pixels
			if(dist > 1)
				numHandRegion++;

			// find max dist value
			if(dist > distMax)
				distMax = dist;

			// increase index
			i += di[k];
			j += dj[k];
			l++;
		} while(l % step != 0);
	}

	// check hand region pixel count
	if(numHandRegion < 1.8 * R || numHandRegion > 3.0 * R)
		return 0;

	// check max dist
	return (1.0 - abs((float)R - distMax) / 2.0 / R);

}

void FingerTip::KeepEndPointHistory()
{
	for(int i = 0; i < _numEndPoint; i++) {
		_KmeansPoint[_nKmeansPointRear].x = _endPoint[i].x;
		_KmeansPoint[_nKmeansPointRear].y = _endPoint[i].y;

		_numKmeansPoint++;
		if(_numKmeansPoint >= MAX_KMEANS_POINT)
			_numKmeansPoint == MAX_KMEANS_POINT;
		_nKmeansPointRear = (_nKmeansPointRear + 1) % MAX_KMEANS_POINT;
		if(_nKmeansPointRear == _nKmeansPointFront)
			_nKmeansPointFront = (_nKmeansPointFront + 1) % MAX_KMEANS_POINT;
	}

}

void FingerTip::Kmeans()
{
	if(_numKmeansPoint < _nCluster * 5)
		return;

	CvMat sampleMat = cvMat(_numKmeansPoint, 1, CV_32FC2, _KmeansPoint);
	CvMat clusterMat = cvMat(_numKmeansPoint, 1, CV_32SC1, _ClusterLabel);

	cvKMeans2(&sampleMat, _nCluster, &clusterMat, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0));

	int _nClusterLabel[MAX_CLUSTER_LABEL];
	for(int i = 0; i < _nCluster; i++) {
		_ClusterCenter[i] = cvPoint2D32f(0, 0);
		_nClusterLabel[i] = 0;
	}
	for(int i = 0; i < _numKmeansPoint; i++) {
		if(_ClusterLabel[i] < 0 || _ClusterLabel[i] >= _nCluster)
			continue;
		_nClusterLabel[_ClusterLabel[i]]++;
		_ClusterCenter[_ClusterLabel[i]].x += _KmeansPoint[i].x;
		_ClusterCenter[_ClusterLabel[i]].y += _KmeansPoint[i].y;
	}
	for(int i = 0; i < _nCluster; i++) {
		if(_nClusterLabel[i] > 0) {
			_ClusterCenter[i].x /= _nClusterLabel[i];
			_ClusterCenter[i].y /= _nClusterLabel[i];
		}
	}
}

void FingerTip::Reset()
{
	_numCirclePoint = 0;
	_numEndPoint = 0;
}

int FingerTip::FindFingerTipCandidatesByCurvature(IplImage * handRegion, bool fScreenshot)
{
	if(!handRegion) {
		return 0;
	}

	if(!_pDistImage) {
		_pDistImage = cvCreateImage(cvGetSize(handRegion), IPL_DEPTH_32F, 1);
		_pDistImage->origin = handRegion->origin;
	}
	if(!_pDistImage2) {
		_pDistImage2 = cvCreateImage(cvGetSize(handRegion), IPL_DEPTH_32F, 1);
		_pDistImage2->origin = handRegion->origin;
	}
	if(!_pHandImage) {
		_pHandImage = cvCreateImage(cvGetSize(handRegion), 8, 1);
		_pHandImage->origin = handRegion->origin;
	}
	IplImage * pScreenshot_curvature = 0;
	IplImage * pScreenshot_ellipse = 0;
	if(fScreenshot) {
		pScreenshot_curvature = cvCreateImage(cvSize(640, 480), 8, 3);
		pScreenshot_curvature->origin = handRegion->origin;
		pScreenshot_ellipse = cvCreateImage(cvSize(640, 480), 8, 3);
		pScreenshot_ellipse->origin = handRegion->origin;
		cvSetZero(pScreenshot_curvature);
		cvSetZero(pScreenshot_ellipse);
	}

	// Get Distance Transform
	if(_numEndPoint == 0) {
		cvDistTransform(handRegion, _pDistImage2, CV_DIST_L2, CV_DIST_MASK_3, NULL, NULL);
		_maxDistValue = 0;
		_maxDistPoint = cvPoint(0, 0);
		cvMinMaxLoc(_pDistImage2, 0, &_maxDistValue, 0, &_maxDistPoint);
		if(_maxDistPoint.x == 0)
			_maxDistPoint.x++;
		if(_maxDistPoint.y == 0)
			_maxDistPoint.y++;
		if(_maxDistPoint.x >= _pDistImage->width - 1)
			_maxDistPoint.x--;
		if(_maxDistPoint.y >= _pDistImage->height- 1)
			_maxDistPoint.y--;
	}

	// Find Contours to get a single connected component for the hand
	// that has the max distance point in it.
	CvMemStorage *pStorage = cvCreateMemStorage(0);
	CvSeq *pContours;   
	int nContours = cvFindContours(handRegion, pStorage, &pContours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,cvPoint(0,0));
	cvSetZero(_pHandImage);
	for(; pContours; pContours = pContours->h_next) {
		if(pContours->total > 100)
			cvDrawContours(_pHandImage, pContours, cvScalar(255), cvScalar(0), 0, -1, 8);
		if(cvGetReal2D(_pHandImage, _maxDistPoint.y, _maxDistPoint.x) == 255) {
			cvSetZero(_pHandImage);
			cvDrawContours(_pHandImage, pContours, cvScalar(255), cvScalar(0), 0, -1, 8);
			cvDistTransform(_pHandImage, _pDistImage, CV_DIST_L2, CV_DIST_MASK_3, NULL, NULL);
			_maxDistValue = 0;
			_maxDistPoint = cvPoint(0, 0);
			cvMinMaxLoc(_pDistImage, 0, &_maxDistValue, 0, &_maxDistPoint);
			if(_maxDistPoint.x == 0)
				_maxDistPoint.x++;
			if(_maxDistPoint.y == 0)
				_maxDistPoint.y++;
			if(_maxDistPoint.x >= _pDistImage->width - 1)
				_maxDistPoint.x--;
			if(_maxDistPoint.y >= _pDistImage->height - 1)
				_maxDistPoint.y--;
			break;
		}
	}
	cvClearMemStorage(pStorage);

	if(pContours == 0)
		return 0;

	// Store the contour as array
	CvSeqReader reader;

	cvStartReadSeq(pContours, &reader, 0);

	CvPoint * prev_pt = (CvPoint *)reader.prev_elem;
	CvPoint * curr_pt = (CvPoint *)reader.ptr;

	// Check minimum contour perimeter
	if(pContours->total < 100) {
		_numEndPoint = 0;
		return 0;
	}

	CvPoint2D32f * contour = (CvPoint2D32f *)malloc(pContours->total * sizeof(CvPoint2D32f));
	for(int i = 0; i < pContours->total; i++) {
		contour[i].x = curr_pt->x;
		contour[i].y = curr_pt->y;

		if(fScreenshot && i > 0)
			cvLine(pScreenshot_ellipse, cvPoint(contour[i-1].x * 2, contour[i-1].y * 2), cvPoint(contour[i].x * 2, contour[i].y * 2), CV_RGB(255, 255, 255), 1, 8, 0);

		// Move to next point
		CV_NEXT_SEQ_ELEM(sizeof(CvPoint), reader);
		prev_pt = curr_pt;
		curr_pt = (CvPoint *)reader.ptr;
	}

	if(fScreenshot)
		cvLine(pScreenshot_ellipse, cvPoint(contour[pContours->total-1].x * 2, contour[pContours->total-1].y * 2), cvPoint(contour[0].x * 2, contour[0].y * 2), CV_RGB(255, 255, 255), 1, 8, 0);

	// Find local maxima curvature points
	_numEndPoint = 0;
	int nPointsConnected = 0;
	int nPointsGap = 0;
	float minAngle = 0;
	int   minPointIndex = -1;
	float mediumIndex = 0;
	float sumAngle = 0;
	float meanPointX = 0;
	float meanPointY = 0;
	int min_curvature_step = _maxDistValue * 0.2;
	int max_curvature_step = _maxDistValue * 1.0;
	if(min_curvature_step < MIN_CURVATURE_STEP) min_curvature_step = MIN_CURVATURE_STEP;
	if(max_curvature_step > MAX_CURVATURE_STEP) max_curvature_step = MAX_CURVATURE_STEP;
	int startIndex, endIndex;
	for(int i = 0; i < pContours->total; i++) {
		// For different scale, calculate curvature
		// _maxDistValue is approximately our bound.
		bool fConnected = false;
		float minK = 0;
		float minDir = 0;
		float count = 0;
		for(int k = min_curvature_step; k <= max_curvature_step; k++) {
			int iPrev = (i - k + pContours->total) % pContours->total;
			int iNext = (i + k) % pContours->total;

			// Compute Curvature
			int pp1_x = contour[iPrev].x - contour[i].x;
			int pp1_y = contour[iPrev].y - contour[i].y;
			int pp2_x = contour[iNext].x - contour[i].x;
			int pp2_y = contour[iNext].y - contour[i].y;
			// dot product (cosine)
			float K = (pp1_x * pp2_x + pp1_y * pp2_y) / (sqrt((float)pp1_x * pp1_x + pp1_y * pp1_y) * sqrt((float)pp2_x*pp2_x + pp2_y*pp2_y));
			// cross product (sign of z-component)
			float dir = pp1_x * pp2_y - pp1_y * pp2_x;

			if(minK <= K) {
				minK = K;
				minDir = dir;
			}
/*
			if(K > FINGERTIP_ANGLE_THRESHOLD) {
				minK += K;
				minDir += dir;
				count += 1;
			}
*/
		}
		minK /= count;
		minDir /= count;

		// take the points based on the curvature and direction
		if(minK > FINGERTIP_ANGLE_THRESHOLD && minDir > 0) {
			nPointsGap = 0;
			if(nPointsConnected == 0) startIndex = i;
			nPointsConnected++;
			if(minAngle < minK) {
				minAngle = minK;
				minPointIndex = i;
			}
			fConnected = true;
			mediumIndex += (minK * i);
			meanPointX += (minK * contour[i].x);
			meanPointY += (minK * contour[i].y);
			sumAngle += minK;
			if(fScreenshot) {
				cvCircle(pScreenshot_curvature, cvPoint(contour[i].x * 2, contour[i].y * 2), 4, CV_RGB(255, 255, 0), -1, 8, 0);
				cvCircle(pScreenshot_ellipse, cvPoint(contour[i].x * 2, contour[i].y * 2), 4, CV_RGB(255, 255, 0), -1, 8, 0);
			}
		}

		if(!fConnected) {
			nPointsGap++;
			if(nPointsGap >= GAP_POINT_THRESHOLD) {
				if(nPointsConnected > CONNECTED_POINT_THRESHOLD) {
					// check image boundary
					if(contour[minPointIndex].x > 10 && contour[minPointIndex].x < _pDistImage->width - 10 && contour[minPointIndex].y > 10 && contour[minPointIndex].y < _pDistImage->height - 10) {
						mediumIndex /= sumAngle;
						meanPointX /= sumAngle;
						meanPointY /= sumAngle;

						// now this is a finger tip
						_endPoint[_numEndPoint].x = meanPointX;
						_endPoint[_numEndPoint].y = meanPointY;

						if(fScreenshot) {
							cvCircle(pScreenshot_curvature, cvPoint(meanPointX * 2, meanPointY * 2), 4, CV_RGB(255, 0, 0), -1, 8, 0);
							cvCircle(pScreenshot_ellipse, cvPoint(meanPointX * 2, meanPointY * 2), 4, CV_RGB(255, 0, 0), -1, 8, 0);
						}
/*
						_endPoint[_numEndPoint].x = contour_x[(int)mediumIndex];
						_endPoint[_numEndPoint].y = contour_y[(int)mediumIndex];
						_endPoint[_numEndPoint].x = contour[minPointIndex].x;
						_endPoint[_numEndPoint].y = contour[minPointIndex].y;
*/

						// Fit an ellipse
						endIndex = i;
						cvFitEllipse(&(contour[startIndex]), endIndex - startIndex + 1, &_Ellipse[_numEndPoint]);
						if(fScreenshot) {
							CvPoint center = cvPoint(_Ellipse[_numEndPoint].center.x * 2, _Ellipse[_numEndPoint].center.y*2);
							CvSize size = cvSize(_Ellipse[_numEndPoint].size.width, _Ellipse[_numEndPoint].size.height);
							if(size.width > 0 && size.height > 0 && size.width < 640 && size.height < 480)
								cvEllipse(pScreenshot_ellipse, center, size, -_Ellipse[_numEndPoint].angle, 0, 360, CV_RGB(0, 255, 0), 2, CV_AA, 0);
						}

						// find the end point along the first axis
						float minDist = 1000000;
						for(int dAngle = 90; dAngle < 360; dAngle += 180) {
							float x1 = _Ellipse[_numEndPoint].center.x;
							float y1 = _Ellipse[_numEndPoint].center.y;
							if(dAngle == 0 || dAngle == 180) {
								x1 += cos((_Ellipse[_numEndPoint].angle + dAngle) * CV_PI / 180) * _Ellipse[_numEndPoint].size.width * 0.5;
								y1 += sin((_Ellipse[_numEndPoint].angle + dAngle) * CV_PI / 180) * _Ellipse[_numEndPoint].size.width * 0.5;
							} else {
								x1 += cos((_Ellipse[_numEndPoint].angle+dAngle) * CV_PI / 180) * _Ellipse[_numEndPoint].size.height * 0.5;
								y1 += sin((_Ellipse[_numEndPoint].angle + dAngle) * CV_PI / 180) * _Ellipse[_numEndPoint].size.height * 0.5;
							}
							float dist = pow(meanPointX-x1, 2) + pow(meanPointY-y1, 2);
							if(minDist > dist) {
#ifdef ELLIPSE_FITTING
								_endPoint[_numEndPoint].x = x1;
								_endPoint[_numEndPoint].y = y1;
#endif
								minDist = dist;
							}
						}
						if(fScreenshot)
							cvCircle(pScreenshot_ellipse, cvPoint(_endPoint[_numEndPoint].x * 2, _endPoint[_numEndPoint].y * 2), 4, CV_RGB(0, 0, 255), -1, 8, 0);

#ifdef PROCESS_320x240
						_endPointDist[_numEndPoint] = 5;
#else
						_endPointDist[_numEndPoint] = 10;
#endif
						_endPointScore[_numEndPoint] = 1;

						_numEndPoint++;
						if(_numEndPoint == MAX_END_POINT)
							_numEndPoint = MAX_END_POINT - 1;

					}
				}
				nPointsConnected = 0;
				minPointIndex = -1;
				minAngle = 0;
				mediumIndex = 0;
				meanPointX = 0;
				meanPointY = 0;
				sumAngle = 0;
			}
		}
	}

	// Save Screenshots
	if(fScreenshot) {
		cvSaveImage("screenshot_curvature.png", pScreenshot_curvature);
		cvSaveImage("screenshot_ellipse.png", pScreenshot_ellipse);
		cvReleaseImage(&pScreenshot_curvature);
		cvReleaseImage(&pScreenshot_ellipse);
	}

	// Free memory
	free(contour);
	cvReleaseMemStorage(&pStorage);

	return _numEndPoint;
}
