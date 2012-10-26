#ifndef _FINGERTIP_TRACKER_H_
#define _FINGERTIP_TRACKER_H_

#include <global.h>

#define NUM_MAX_CANDIDATES		20
#define THRESHOLD_CLOSEST_DIST		50
#define THRESHOLD_LOST_TRACKING		5
#define MIN_AGE_TO_TRACK		5
#define THRESHOLD_AGE_TO_DETECT		10

#define MAX_TEMPLATE_SIZE		75
#define TEMPLATE_SEARCH_WINDOW		100
#define THRESHOLD_TEMPLATE_MATCH	0.8
#define THRESHOLD_CLOSEST_DIST2		20

#define NUM_FINGERTIP			5

#define MODE_FINGERTIP_NONE		0
#define MODE_FINGERTIP_FIRST_DETECTED	1
#define MODE_FINGERTIP_TRACKING		2
#define MODE_FINGERTIP_LOST_TRACKING	3

class FingertipTracker
{
public:
	FingertipTracker(void);
	~FingertipTracker(void);

	bool LoadFingertipCoordinates(char *filename);
	bool SaveFingertipCoordinates(char *filename);

	void Reset();
	bool FeedFingertipCandidates(IplImage *image, int nPoints, CvPoint *points, float *distValue, CvPoint currCentroid);
	bool TrackFingertips(IplImage *image, int nPoints, CvPoint *points, float *distValue);
	bool TrackFingertips2(int nPoints, CvPoint *points, float *distValue, CvPoint prevCentroid, CvPoint currCentroid);
	bool TrackFingertips3(int nPoints, CvPoint *points, float *distValue, CvPoint prevCentroid, CvPoint currCentroid);

	CvPoint2D32f *QueryFingertip(int index);
	CvPoint2D32f   QueryFingertipKalman(int index);
	float QueryFingertipDist(int index);

	void PredictFingertips(const CvMat *intrinsic_matrix, const CvMat *distortion_coeffs, const CvMat *rotation_vector, const CvMat *translation_vector);
	bool FindExtrinsicCameraParams(const CvMat *intrinsic_matrix, const CvMat *distortion_coeffs, CvMat *rotation_vector, CvMat *translation_vector);

	void ResetCoordinates();
	bool TriangulateCoordinates(const CvMat *ProjectionMat);
	bool CalculateCoordinates(const CvMat *ProjectionMat);

	void CentralizeCoordinates();
	float *QueryCenterCoordinates();

	int QueryTrackingMode() { return   _nMode; }
	bool QueryFlipOrder() { return   _fFlipOrder; }

protected:
	void MakeFingertipTemplate(IplImage *image, CvPoint point, float dist, IplImage **pTemplate);
	void MatchFingertipTemplate(IplImage *image);
	void MatchCorrespondencesByNearestNeighbor(int nPoints, CvPoint *points, float *distValue);

private:
	// Tracking Mode
	int _nMode;

	// Candidates
	//
	int _nPoints;
	CvPoint2D32f _candidatePoints[NUM_MAX_CANDIDATES];
	int _candidateScore[NUM_MAX_CANDIDATES];
	int _candidateAge[NUM_MAX_CANDIDATES];
	int _candidateLost[NUM_MAX_CANDIDATES];
	float _candidateVelocity[NUM_MAX_CANDIDATES][2];
	float _candidateDist[NUM_MAX_CANDIDATES];

	// Fingertips
	bool _fDetected;
	int _nFingertips;
	int _nFingertipIndex[NUM_FINGERTIP];	// fingertip --> candidate
	CvPoint2D32f _fingertipPoints[NUM_FINGERTIP];
	float _fingertipDist[NUM_FINGERTIP];
	IplImage *_pFingertipTemplate[NUM_FINGERTIP];
	bool _fTracked[NUM_FINGERTIP];
	bool _fFlipOrder;

	// Fingertip Coordinates
	bool _fFingertipCoordinates;
	float _fingertipCoordinates[NUM_FINGERTIP][3];
	float _numCoordinateSamples;
	FILE *_fpCoordinates;

	bool _fInitialFingertip;
	CvPoint2D32f _InitialFingertipPoints[NUM_FINGERTIP];
	float _InitialProjectionMatrix[3][4];

	float _centerCoordinates[3];

/*
	// Kalman filters for each fingertip
	//  x(t+1) = A x(t) + G w(t)
	//  y(t)   = H x(t) + v(t)
	CvRandState _Rng;
	CvKalman *_pKalman	[NUM_MAX_CANDIDATES];
	CvMat *_pKalmanMat   _A;  // State Transition Matrix
	CvMat *_pKalmanMat   _G;  // Driving Matrix
	CvMat *_pKalmanMat   _H;  // Observation Matrix
	CvMat *_pKalmanMat   _W;  // Process Noise Matrix
*/

};

#endif // _FINGERTIP_TRACKER_H_

