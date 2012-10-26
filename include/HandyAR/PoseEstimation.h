#ifndef _POSE_ESTIMATION_H_
#define _POSE_ESTIMATION_H_

#include <global.h>

#define NUM_CHESS_POINT		48
#define NUM_CHESS_ROW		8
#define NUM_CHESS_COL		6
#define CHESS_CORNER_SIZE_MM	30.0

class PoseEstimation
{
public:
	PoseEstimation(void);
	~PoseEstimation(void);

	bool ReadIntrinsicParameters(char *filename);

	bool FindExtrinsicFromChessboard(IplImage *colorImage, IplImage *grayImage);

	float ComputeReprojectionError(IplImage *colorImage, CvMat *rotation_vec, CvMat *translation_vec, bool fScreenshot = false);
	float ComputeReprojectErroForARTag(IplImage *colorImage, float reproj_pt[][2], bool fScreenshot = false);

	CvMat *QueryRotationMat() { return &rotation; }
	CvMat *QueryTranslationMat() { return &translation; }

	float QueryProjection(int i, int j) { return projection_matrix[i][j]; }
	CvMat *QueryProjectionMat() { return &P; }

private:

	// Intrinsic Parameters
	bool  _fIntrinsic;
	float camera_intrinsic[3][3];
	float camera_distortion[4];
	float camera_rotation[3];
	float camera_translation[3];
	CvMat intrinsic;
	CvMat distortion;
	CvMat rotation;
	CvMat translation;
	float camera_rotation3by3[3][3];
	CvMat rotation3by3;

	float projection_matrix[3][4];
	CvMat P;

	// chess points
	CvPoint2D32f chess_pt[NUM_CHESS_POINT];
};

#endif // _POSE_ESTIMATION_H_
