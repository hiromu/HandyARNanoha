#ifndef _FINGERTIP_POSE_ESTIMATION_H_
#define _FINGERTIP_POSE_ESTIMATION_H_

#include <global.h>
#include "HandRegion.h"
#include "FingerTip.h"
#include "FingertipTracker.h"
#include "PoseEstimation.h"
#include "ColorDistribution.h"

class FingertipPoseEstimation
{
public:
	FingertipPoseEstimation(void);
	~FingertipPoseEstimation(void);

	bool Initialize(IplImage *srcImage, char *calibFilename);
	void Terminate();

	void Reset();
	void OnCapture(IplImage *frame, int64 nTickCount, IplImage *gray = 0);
	void OnProcess();
	IplImage *OnDisplay(IplImage *image = 0);

	bool LoadFingertipCoordinates(char *filename);
	bool SaveFingertipCoordinates(char *filename);
	bool ToggleChessboard();
	bool ToggleBuildHandModel();

	bool  QueryValidPose();
	CvMat QueryRotationMat();
	CvMat QueryTranslationMat();
	float *QueryModelViewMat();
	float QueryR(int i, int j) { return _FingerRotation3by3[i][j]; }
	float QueryT(int i) { return _FingerTranslation[i]; }
	float QueryCR(int i, int j) { return _CameraCenterR[i][j]; }
	float QueryCT(int i) { return _CameraCenterT[i]; }

	void SetOpenGLFrustrum();
	void SetOpenGLModelView();
	void RenderCaptureImage();

	float QueryFx();
	float QueryFy();

	void SetARTagChessboardFound(bool fFound, float chess_pt[][2] = NULL);
	float **QueryARTagChessboard();

	float QueryDeltaT();
	int QueryCentroidX();
	int QueryCentroidY();

	void PrintScreenshot();

	void TickCountBegin();
	void TickCountEnd();
	void TickCountNewLine();
	void TickCountDummy();

private:
	// Images
	IplImage *_pImage;
	IplImage *_pGray;
	IplImage *_pImage320x240;
	IplImage *_pSkinColorImage;

	CvFont font;

	// Time stamp
	int64 _nTickCount_Prev;
	int64 _nTickCount_Curr;
	double _deltaT;

	// Process Modules
	HandRegion _HandRegion;
	FingerTip _FingerTip;
	FingertipTracker _FingertipTracker;
	PoseEstimation _PoseEstimation;
	ColorDistribution _ColorDistribution;

	// Hand Center
	CvPoint _PrevCentroid;
	CvPoint _CurrCentroid;

	// Running Mode
	int _nRunningMode;
	bool _fHandRegion;
	bool _fDistTrans;
	bool _fFingertipDetected;
	bool _fFindChessboard;
	bool _fBuildHandModel;

	// Intrinsic Camera Parameters
	float _CameraIntrinsic[3][3];
	float _CameraDistortion[4];
	CvMat _IntrinsicMat;
	CvMat _DistortionMat;

	// Pose Estimation by Chessboard
	bool _fPoseEstimatedByChessboard;
	float _ChessRotation[3];
	float _ChessTranslation[3];
	CvMat _ChessRotationMat;
	CvMat _ChessTranslationMat;

	// Pose Estimation by Fingertips
	bool _fPoseEstimatedByFingertips;
	bool _fValidPose;
	float _FingerRotation[3];
	float _FingerRotation3by3[3][3];
	float _FingerTranslation[3];
	float _FingerQuaternion[4];
	CvMat _FingerRotationMat;
	CvMat _FingerRotation3by3Mat;
	CvMat _FingerTranslationMat;
	CvMat _FingerQuaternionMat;

	// Camera Transform
	float _CameraCenterT[3];
	float _CameraCenterR[3][3];
	CvMat _CameraCenterTMat;
	CvMat _CameraCenterRMat;

	// Difference
	float _CenterTDiff[3];
	FILE *_fpDiff;

	// OpenGL View Frustrum
	GLuint _nCameraTexID;

	// OpenGL ModelView Matrix
	float _ModelView[16];

	// Kalman filter for the camera pose
	// x(t+1) = A x(t) + G w(t)
	// y(t)   = H x(t) +   v(t)
	CvRandState _Rng;
	CvKalman *_pKalman;    // Kalman filter for the camera pose
	CvMat *_pKalmanMat_A;  // State Transition Matrix
	CvMat *_pKalmanMat_G;  // Driving Matrix
	CvMat *_pKalmanMat_H;  // Observation Matrix
	CvMat *_pKalmanMat_W;  // Process Noise Matrix

	// Chessboard points reproject by ARTag
	bool _artag_chess_found;
	float _artag_chess_pt[NUM_CHESS_ROW *NUM_CHESS_COL][2];

	// Print Screenshot
	bool _fScreenshot;

	// Processing Time Profile
	FILE *_fpTime;
	int64 _PrevTickCount;

};

#endif // _FINGERTIP_POSE_ESTIMATION_H_
