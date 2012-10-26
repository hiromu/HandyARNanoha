#include <HandyAR/PoseEstimation.h>

PoseEstimation::PoseEstimation(void)
{
	intrinsic = cvMat(3, 3, CV_32FC1, camera_intrinsic);
	distortion = cvMat(4, 1, CV_32FC1, camera_distortion);
	rotation = cvMat(3, 1, CV_32FC1, camera_rotation);
	translation = cvMat(3, 1, CV_32FC1, camera_translation);

	rotation3by3 = cvMat(3, 3, CV_32FC1, camera_rotation3by3);

	P = cvMat(3, 4, CV_32FC1, projection_matrix);

	for(int i = 0 ; i < 3 ; i++) {
		camera_rotation[i] = 0;
		camera_translation[i] = 0;
	}

	_fIntrinsic = false;

}

PoseEstimation::~PoseEstimation(void)
{
}

bool PoseEstimation::ReadIntrinsicParameters(char *filename)
{
	FILE *fp = fopen(filename, "rt");
	if(fp == NULL)
		return false;

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			camera_intrinsic[i][j] = 0;
	camera_intrinsic[2][2] = 1;

	fscanf(fp, "%f", &(camera_intrinsic[0][0]));
	fscanf(fp, "%f", &(camera_intrinsic[1][1]));
	fscanf(fp, "%f", &(camera_intrinsic[0][2]));
	fscanf(fp, "%f", &(camera_intrinsic[1][2]));

	for(int i=0; i<4; i++)
		fscanf(fp, "%f", &(camera_distortion[i]));
	fclose(fp);

	printf("fx=%.5f fy=%.5f cx=%.5f cy=%.5f\n", camera_intrinsic[0][0], camera_intrinsic[1][1], camera_intrinsic[0][2], camera_intrinsic[1][2]);
	printf("k1=%.5f k2=%.5f p1=%.5f p2=%.5f\n", camera_distortion[0], camera_distortion[1], camera_distortion[2], camera_distortion[3]);

	_fIntrinsic = true;

	return true;
}

bool PoseEstimation::FindExtrinsicFromChessboard(IplImage *colorImage, IplImage *grayImage)
{
	// Find Chessboard
	//CvPoint2D32f chess_pt[NUM_CHESS_POINT];
	int chess_count = 0;
	bool fChessboardFound = cvFindChessboardCorners(grayImage, cvSize(NUM_CHESS_COL, NUM_CHESS_ROW), chess_pt, &chess_count);

	if(fChessboardFound) {
		cvFindCornerSubPix(grayImage, chess_pt, chess_count, cvSize(10, 10), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.02));
		cvDrawChessboardCorners(colorImage, cvSize(NUM_CHESS_COL, NUM_CHESS_ROW), chess_pt, chess_count, fChessboardFound);
	}

	if(chess_count != NUM_CHESS_POINT)
		return false;

	// Prepare Chessboard Points

	// object points
	float calib_obj_pt[NUM_CHESS_POINT][3];
	CvMat obj_pt = cvMat(chess_count, 3, CV_32FC1, calib_obj_pt);
	int k = 0;
	for(int i = 0; i < NUM_CHESS_ROW; i++) {
		for(int j = 0; j < NUM_CHESS_COL; j++) {
			calib_obj_pt[k][0] = (float)CHESS_CORNER_SIZE_MM * j;
			calib_obj_pt[k][1] = (float)CHESS_CORNER_SIZE_MM * i;
			calib_obj_pt[k][2] = 1;
			k++;
		}
	}

/*
	calib_obj_pt[0][0] = calib_obj_pt[2][0];
	calib_obj_pt[0][1] = calib_obj_pt[2][1];
	calib_obj_pt[1][0] = calib_obj_pt[4][0];
	calib_obj_pt[1][1] = calib_obj_pt[4][1];
	calib_obj_pt[2][0] = calib_obj_pt[8][0];
	calib_obj_pt[2][1] = calib_obj_pt[8][1];
	calib_obj_pt[3][0] = calib_obj_pt[21][0];
	calib_obj_pt[3][1] = calib_obj_pt[21][1];
	calib_obj_pt[4][0] = calib_obj_pt[45][0];
	calib_obj_pt[4][1] = calib_obj_pt[45][1];
	obj_pt = cvMat(5, 3, CV_32FC1, calib_obj_pt);
*/

	// image points
	float calib_img_pt[NUM_CHESS_POINT][2];
	CvMat img_pt = cvMat(chess_count, 2, CV_32FC1, calib_img_pt);
	for(int i=0; i<chess_count; i++) {
		calib_img_pt[i][0] = chess_pt[i].x;
		calib_img_pt[i][1] = chess_pt[i].y;
	}

/*
	calib_img_pt[0][0] = calib_img_pt[2][0];
	calib_img_pt[0][1] = calib_img_pt[2][1];
	calib_img_pt[1][0] = calib_img_pt[4][0];
	calib_img_pt[1][1] = calib_img_pt[4][1];
	calib_img_pt[2][0] = calib_img_pt[8][0];
	calib_img_pt[2][1] = calib_img_pt[8][1];
	calib_img_pt[3][0] = calib_img_pt[21][0];
	calib_img_pt[3][1] = calib_img_pt[21][1];
	calib_img_pt[4][0] = calib_img_pt[45][0];
	calib_img_pt[4][1] = calib_img_pt[45][1];
	img_pt = cvMat(5, 2, CV_32FC1, calib_img_pt);
*/

	// Call OpenCV Calibration for extrinsic parameters
/*
	if(cvFindExtrinsicCameraParams3(&obj_pt, &img_pt, &intrinsic, &distortion, &rotation, &translation) == false)
	   return false;
*/

	cvFindExtrinsicCameraParams2(&obj_pt, &img_pt, &intrinsic, &distortion, &rotation, &translation);

	// rotation
	cvRodrigues2(&rotation, &rotation3by3);

	// camera center
/*
	float r_[3][3];
	CvMat r_inv = cvMat(3, 3, CV_32FC1, r_);
	cvInvert(&rotation3by3, &r_inv);
	CvMat C = cvMat(3, 1, CV_32FC1, camera_center);
	cvMatMul(&r_inv, &translation, &C);
	camera_center[0] *= -1;
	camera_center[1] *= -1;
	camera_center[2] *= -1;
	printf("Camera = (%5.2f, %5.2f, %5.2f) (from chessboard)\n", camera_center[0], camera_center[1], camera_center[2]);
	printf("\n");
	// camera orientation
	float dir[3] = { 0, 0, 1 };
	CvMat Dir = cvMat(3, 1, CV_32FC1, dir);
	CvMat D = cvMat(3, 1, CV_32FC1, camera_dir);
	cvMatMul(&rotation3by3, &Dir, &D);
*/

	// Projection Matrix
	CvMat K = cvMat(3, 3, CV_32FC1, camera_intrinsic);
	float rt[3][4];
	CvMat RT = cvMat(3, 4, CV_32FC1, rt);
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			rt[i][j] = camera_rotation3by3[i][j];
		rt[i][3] = camera_translation[i];
	}
	cvMatMul(&K, &RT, &P);

	return true;
}

float PoseEstimation::ComputeReprojectionError(IplImage *colorImage, CvMat *rotation_vec, CvMat *translation_vec, bool fScreenshot) {
	// Prepare obj points
	float calib_obj_pt[NUM_CHESS_POINT][3];
	CvMat obj_pt = cvMat(NUM_CHESS_POINT, 3, CV_32FC1, calib_obj_pt);
	int k = 0;
	for(int i = 0; i < NUM_CHESS_ROW; i++) {
		for(int j = 0; j < NUM_CHESS_COL; j++) {
			calib_obj_pt[k][0] = (float)CHESS_CORNER_SIZE_MM * j;
			calib_obj_pt[k][1] = (float)CHESS_CORNER_SIZE_MM * i;
			calib_obj_pt[k][2] = 1;
			k++;
		}
	}

	// Project the points
	float reprojection_pt[NUM_CHESS_POINT][2];
	CvMat reprojectionPointsMat = cvMat(NUM_CHESS_POINT, 2, CV_32FC1, reprojection_pt);
	cvProjectPoints2(&obj_pt, rotation_vec, translation_vec, &intrinsic, &distortion, &reprojectionPointsMat);

	// Capture screenshot
	if(fScreenshot) {
		IplImage *pScreenshot = cvCreateImage(cvGetSize(colorImage), 8, 3);
		pScreenshot->origin = colorImage->origin;
		cvSet(pScreenshot, CV_RGB(255, 255, 255));

		for(int i = 0 ; i < NUM_CHESS_POINT ; i++)
			cvCircle(pScreenshot, cvPoint(reprojection_pt[i][0], reprojection_pt[i][1]), 3, CV_RGB(0, 0, 255), -1, 8, 0);
		cvSaveImage("screenshot_reproj_finger.png", pScreenshot);
		cvReleaseImage(&pScreenshot);
	}

	// Find Chessboard points
/*
	CvPoint2D32f chess_pt[NUM_CHESS_POINT];
	int chess_count = 0;
	bool fChessboardFound = cvFindChessboardCorners(grayImage, cvSize(NUM_CHESS_COL, NUM_CHESS_ROW), chess_pt, &chess_count);

	float error = -1;
	if(fChessboardFound) {
	      cvFindCornerSubPix(grayImage, chess_pt, chess_count, cvSize(10, 10), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.02));
*/

		// Compute rms error
		float error = -1;
		for(int i =0 ; i < NUM_CHESS_POINT ; i++) {
			error += pow(reprojection_pt[i][0] - chess_pt[i].x, 2);
			error += pow(reprojection_pt[i][1] - chess_pt[i].y, 2);
			cvCircle(colorImage, cvPoint(reprojection_pt[i][0], reprojection_pt[i][1]), 10, CV_RGB(0, 0, 255), 1, 8, 0);
		}
		error /= NUM_CHESS_POINT;
		error = sqrt(error);
//	}
	return error;
}

float PoseEstimation::ComputeReprojectErroForARTag(IplImage * colorImage, float reproj_pt[][2], bool fScreenshot)
{
	// Capture screenshot
	if(fScreenshot) {
		IplImage * pScreenshot = cvCreateImage(cvGetSize(colorImage), 8, 3);
		pScreenshot->origin = colorImage->origin;
		cvSet(pScreenshot, CV_RGB(255, 255, 255));

		for(int i = 0 ; i < NUM_CHESS_POINT ; i++)
			cvCircle(pScreenshot, cvPoint(reproj_pt[i][0], reproj_pt[i][1]), 3, CV_RGB(255, 0, 0), -1, 8, 0);
		cvSaveImage("screenshot_reproj_artag.png", pScreenshot);
		cvReleaseImage(&pScreenshot);
	}

	// Compute rms error
	float error = -1;
	for(int i =0 ; i < NUM_CHESS_POINT ; i++) {
		error += pow(reproj_pt[i][0] - chess_pt[i].x, 2);
		error += pow(reproj_pt[i][1] - chess_pt[i].y, 2);
		cvCircle(colorImage, cvPoint(reproj_pt[i][0], reproj_pt[i][1]), 10, CV_RGB(255, 0, 0), 1, 8, 0);
	}
	error /= NUM_CHESS_POINT;
	error = sqrt(error);

	return error;
}
