#ifndef _FINGER_TIP_H_
#define _FINGER_TIP_H_

#include <global.h>

#define MAX_CIRCLE_POINT    100000
#define MAX_END_POINT       1000
#define MAX_FINGER_POINT    20
#define MAX_KMEANS_POINT    100
#define MAX_CLUSTER_LABEL   10

#define FINGERTIP_CONDITION_RANGE   1.5

#ifdef PROCESS_320x240
    #define MIN_CURVATURE_STEP          5
    #define MAX_CURVATURE_STEP          25
    #define FINGERTIP_ANGLE_THRESHOLD   0.5
    #define GAP_POINT_THRESHOLD         5
    #define CONNECTED_POINT_THRESHOLD   5
#else
    #define MIN_CURVATURE_STEP          10
    #define MAX_CURVATURE_STEP          50
    #define FINGERTIP_ANGLE_THRESHOLD   0.5
    #define GAP_POINT_THRESHOLD         10
    #define CONNECTED_POINT_THRESHOLD   10
#endif

class FingerTip
{
public:
	FingerTip(void);
	~FingerTip(void);

	void Reset();

	int FindFingerTipCandidates(IplImage *handRegion);
	int FindFingerTipCandidatesByTemplateMatching(IplImage *handRegion);
	int FindFingerTipCandidatesByCurvature(IplImage *handRegion, bool fScreenshot = false);

	void KeepEndPointHistory();
	void Kmeans();
	void TrackFingerTips();

	CvPoint QueryEndPoint(int index) { return _endPoint[index]; }
	float   QueryEndPointDist(int index) { return _endPointDist[index]; }
	CvPoint QueryClusterCenterPoint(int index) { return cvPoint(_ClusterCenter[index].x, _ClusterCenter[index].y); }

private:
	// Find FingerTips by Distance Transform
	void FindSecondRankPoints();
	int  GetRankInNeighbor(int x, int y);
	void FindEndPoints(int index, int prevIndex, int prevIndex2);
	void FloodFill(IplImage *src, IplImage *dst, CvPoint point);

	// Fingertip conditions
	bool CheckFingerTipCondition(IplImage *image, int x, int y, int size);
	float GetFingerTipScore(IplImage *distImage, int x, int y);

	void AddFingerTip(int index);
	void RemoveAdjacentEndPoints();
	void SortEndPoints();

public:
	// Single Connected Component for Hand
	IplImage *_pHandImage;

	// Distance Transform
	IplImage *_pDistImage;  // Distance Transform Image (only for a hand)
	IplImage *_pDistImage2; // Temporary Distance Transform (for all image)
	CvPoint _maxDistPoint;
	double _maxDistValue;

	// Templates
	IplImage *_circleTemplate5;
	IplImage *_circleMatch5;

	// Circle Points (Second Rank Points)
	int _numCirclePoint;
	bool _circleVisited[MAX_CIRCLE_POINT];
	float _circleDist[MAX_CIRCLE_POINT];
	CvPoint _circlePoint[MAX_CIRCLE_POINT];
	int _circleMaxDistIndex;
	float _circleMaxDistValue;

	// End Points (FingerTip Candidates)
	int _numEndPoint;
	float _endPointDist[MAX_END_POINT];
	CvPoint _endPoint[MAX_END_POINT];
	float _endPointScore[MAX_END_POINT];
	int _endPointNumAdj[MAX_END_POINT];

	// Ellipses
	CvBox2D _Ellipse[MAX_END_POINT];

	// Kalman Filters for Finger Points
	int _numFingerPoint;
	CvKalman *_pKalman[MAX_FINGER_POINT];

	// K-means
	int _numKmeansPoint;
	int _nKmeansPointFront;
	int _nKmeansPointRear;
	CvPoint2D32f _KmeansPoint[MAX_KMEANS_POINT];
	int _nCluster;
	int _ClusterLabel[MAX_KMEANS_POINT];
	CvPoint2D32f _ClusterCenter[MAX_CLUSTER_LABEL];
};

#endif // _FINGER_TIP_H_
