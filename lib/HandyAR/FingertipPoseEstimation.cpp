#include <HandyAR/FingertipPoseEstimation.h>
#include <HandyAR/quaternion.h>

FingertipPoseEstimation::FingertipPoseEstimation(void)
{

    // Images
    _pImage = 0;
    _pGray = 0;
    _pImage320x240 = 0;
    _pSkinColorImage = 0;

    // Running Mode
    _nRunningMode = 0;
    _fHandRegion = false;
    _fDistTrans = false;
    _fFingertipDetected = false;
    _fFindChessboard = false;
    _fBuildHandModel = false;

    // Intrinsic Camera Parameters
    _IntrinsicMat = cvMat( 3, 3, CV_32FC1, _CameraIntrinsic );
    _DistortionMat = cvMat( 4, 1, CV_32FC1, _CameraDistortion );

    // Pose Estimation by Chessboard
    _ChessRotationMat = cvMat( 3, 1, CV_32FC1, _ChessRotation );
    _ChessTranslationMat = cvMat( 3, 1, CV_32FC1, _ChessTranslation );

    //
    // Pose Estimation by Fingertips
    //
    _fValidPose = false;
    _FingerRotationMat = cvMat( 3, 1, CV_32FC1, _FingerRotation );
    _FingerRotation3by3Mat = cvMat( 3, 3, CV_32FC1, _FingerRotation3by3 );
    _FingerTranslationMat = cvMat( 3, 1, CV_32FC1, _FingerTranslation );
    _FingerQuaternionMat = cvMat( 4, 1, CV_32FC1, _FingerQuaternion );

    //
    // Camera Transform
    //
    _CameraCenterTMat = cvMat( 3, 1, CV_32FC1, _CameraCenterT );
    _CameraCenterRMat = cvMat( 3, 3, CV_32FC1, _CameraCenterR );

    //
    // Diff file
    //
    _fpDiff = 0;

    // Kalman filter
    _pKalman = 0;
    _pKalmanMat_A = 0;
    _pKalmanMat_G = 0;
    _pKalmanMat_H = 0;
    _pKalmanMat_W = 0;

    // ARTag
    _artag_chess_found = false;

    // Screenshot
    _fScreenshot = false;

    // Time Profile
    _fpTime = 0;
}

FingertipPoseEstimation::~FingertipPoseEstimation(void)
{
    Terminate();
}

bool FingertipPoseEstimation::Initialize( IplImage * srcImage, char * calibFilename )
{
    bool fResult = false;
{
    //
    // Font
    //
    cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, 8 );

    //
    // Images
    //
    if ( !_pImage )
    {
        _pImage = cvCreateImage( cvGetSize( srcImage ), 8, 3 );
        _pImage->origin = srcImage->origin;

        _pGray = cvCreateImage( cvGetSize( _pImage ), 8, 1 );
        _pGray->origin = _pImage->origin;

        _pImage320x240 = cvCreateImage( cvSize( 320, 240 ), 8, 3 );
        _pImage->origin = srcImage->origin;
#ifdef PROCESS_320x240
        _pSkinColorImage = cvCreateImage( cvSize( 320, 240 ), 8, 1 );
#else
        _pSkinColorImage = cvCreateImage( cvGetSize( _pImage ), 8, 1 );
#endif
        _pSkinColorImage->origin = srcImage->origin;
    }

    //
    // Initialize Process Modules
    //

#ifdef USE_GAUSSIAN_MODEL

    // Hand Region
    if ( _HandRegion.LoadSkinColorProbTable() == false )
    {
        goto Finished;
    }

#endif

    // Hand Mask
#ifdef PROCESS_320x240
    if ( _HandRegion.InitHandMask( _pImage320x240 ) == false )
    {
        goto Finished;
    }
#else
    if ( _HandRegion.InitHandMask( srcImage ) == false )
    {
        goto Finished;
    }
#endif

    if ( _HandRegion.LoadColorConfig() == false )
    {
        goto Finished;
    }

    // Intrinsic Parameters ( for Chessboard )
    if ( _PoseEstimation.ReadIntrinsicParameters( calibFilename ) == false )
    {
        goto Finished;
    }

    // Intrinsic Parameters ( for Fingertips, I know, duplicated, but i'm lazy )
	FILE *fp = fopen( calibFilename, "rt");
	if (fp == NULL)
    {
        goto Finished;
    }

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			_CameraIntrinsic[i][j] = 0;
	_CameraIntrinsic[2][2] = 1;
	fscanf(fp, "%f", &(_CameraIntrinsic[0][0]));
	fscanf(fp, "%f", &(_CameraIntrinsic[1][1]));
	fscanf(fp, "%f", &(_CameraIntrinsic[0][2]));
	fscanf(fp, "%f", &(_CameraIntrinsic[1][2]));

	for (int i=0; i<4; i++)
		fscanf(fp, "%f", &(_CameraDistortion[i]));
	fclose(fp);

    //
    // Diff File
    //
    _fpDiff = fopen( "diff.txt", "wt" );
    if ( !_fpDiff )
    {
        goto Finished;
    }

    //
    // Texture for Video Frame
    //
    glGenTextures( 1, &_nCameraTexID );
    glBindTexture( GL_TEXTURE_2D, _nCameraTexID );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, 0 );

    //
    // Kalman filter
    //
    cvRandInit( &_Rng, 0, 1, -1, CV_RAND_UNI );
    _pKalman = 0;
    _pKalmanMat_A = cvCreateMat( 14, 14, CV_32FC1 );
    cvSetIdentity( _pKalmanMat_A );
    for ( int i = 0 ; i < 7 ; i ++ )
        cvSetReal2D( _pKalmanMat_A, i, i+7, 1 );

    _pKalmanMat_G = cvCreateMat( 14, 7, CV_32FC1 );
    cvSetZero( _pKalmanMat_G );
    for ( int i = 0 ; i < 7 ; i ++ )
        cvSetReal2D( _pKalmanMat_G, i+7, i, 1 );

    _pKalmanMat_H = cvCreateMat( 7, 14, CV_32FC1 );
    cvSetZero( _pKalmanMat_H );
    for ( int i = 0 ; i < 7 ; i ++ )
        cvSetReal2D( _pKalmanMat_H, i, i, 1 );

    _pKalmanMat_W = cvCreateMat( 14, 1, CV_32FC1 );

    //
    // Time Profile
    //
    _fpTime = fopen( "time.txt", "wt" );

    //
    // done
    //
    fResult = true;
}
Finished:

    return fResult;
}

void FingertipPoseEstimation::Terminate()
{
    if ( _pImage )
    {
        cvReleaseImage( &_pImage );
        _pImage = 0;
    }
    if ( _pGray )
    {
        cvReleaseImage( &_pGray );
        _pGray = 0;
    }
    if ( _pImage320x240 )
    {
        cvReleaseImage( &_pImage320x240 );
        _pImage320x240 = 0;
    }
    if ( _pSkinColorImage )
    {
        cvReleaseImage( &_pSkinColorImage );
        _pSkinColorImage = 0;
    }
    if ( _fpDiff )
    {
        fclose( _fpDiff );
        _fpDiff = 0;
    }
    if ( _pKalman )
    {
        cvReleaseKalman( &_pKalman );
        _pKalman = 0;
    }
    if ( _pKalmanMat_A )
    {
        cvReleaseMat( &_pKalmanMat_A );
        _pKalmanMat_A = 0;
    }
    if ( _pKalmanMat_G )
    {
        cvReleaseMat( &_pKalmanMat_G );
        _pKalmanMat_G = 0;
    }
    if ( _pKalmanMat_H )
    {
        cvReleaseMat( &_pKalmanMat_H );
        _pKalmanMat_H = 0;
    }
    if ( _pKalmanMat_W )
    {
        cvReleaseMat( &_pKalmanMat_W );
        _pKalmanMat_W = 0;
    }
    if ( _fpTime )
    {
        fclose( _fpTime );
        _fpTime = 0;
    }
}

void FingertipPoseEstimation::Reset()
{
    //
    // Reset color learning, detecting, and tracking
    //
    _HandRegion.LearnColorCrCb( false );
    _fFingertipDetected = false;
    _FingertipTracker.Reset();
    // Reset Kalman filter
    if ( _pKalman )
    {
        cvReleaseKalman( &_pKalman );
        _pKalman = 0;
    }

}

void FingertipPoseEstimation::TickCountBegin()
{
    _PrevTickCount = cvGetTickCount();
}

void FingertipPoseEstimation::TickCountEnd()
{
    int64 currTickCount = cvGetTickCount();
    double timeSpent = ( currTickCount - _PrevTickCount ) / cvGetTickFrequency();
    _PrevTickCount = currTickCount;

    fprintf( _fpTime, "%.5f\t", timeSpent );
}

void FingertipPoseEstimation::TickCountNewLine()
{
    fprintf( _fpTime, "\n" );
}

void FingertipPoseEstimation::TickCountDummy()
{
    fprintf( _fpTime, " \t" );
}

void FingertipPoseEstimation::OnCapture( IplImage * frame, int64 nTickCount, IplImage * gray )
{
    //
    // uhoh, let me do a little better way of this.
    // maybe we just don't need to copy when we have already another
    // copy of the images. anyway, i am gonna do this for now.
    //
    cvCopy( frame, _pImage );
    cvConvertImage( _pImage, _pGray, CV_BGR2GRAY );
#ifdef PROCESS_320x240
    cvPyrDown( frame, _pImage320x240 );
#endif

    //
    // Update time stamp
    //
    _nTickCount_Prev = _nTickCount_Curr;
    _nTickCount_Curr = nTickCount;
    _deltaT = ( _nTickCount_Curr - _nTickCount_Prev ) / cvGetTickFrequency() / 1000000.0;

}

void FingertipPoseEstimation::OnProcess()
{
    // Screenshot ( capture frame )
    if ( _fScreenshot )
    {
        cvSaveImage( "screenshot_captured.png", _pImage );
    }

    //
    // Chessboard Pose Estimation
    //
    float chess_center[3];
    CvMat ChessC = cvMat( 3, 1, CV_32FC1, chess_center );

    if ( _fFindChessboard )
    {
        _fPoseEstimatedByChessboard = _PoseEstimation.FindExtrinsicFromChessboard( _pImage, _pGray );
        _ChessRotationMat    = * _PoseEstimation.QueryRotationMat();
        _ChessTranslationMat = * _PoseEstimation.QueryTranslationMat();
        if ( _fPoseEstimatedByChessboard == false )
        {
            cvSetZero( &_ChessRotationMat );
            cvSetZero( &_ChessTranslationMat );
        }
        else
        {
            float camera_rotation3by3[3][3];
            CvMat rotation3by3 = cvMat(3, 3, CV_32FC1, camera_rotation3by3);
            cvRodrigues2(&_ChessRotationMat, &rotation3by3);
            float r_[3][3];
            CvMat r_inv = cvMat(3, 3, CV_32FC1, r_);
            cvInvert(&rotation3by3, &r_inv);
            cvMatMul(&r_inv, &_ChessTranslationMat, &ChessC);
            cvScale( &ChessC, &ChessC, -1 );
            printf("Camera(by Chessboard) = (%5.2f, %5.2f, %5.2f)\n", chess_center[0], chess_center[1], chess_center[2]);
        }
    }
    else
    {
        _fPoseEstimatedByChessboard = false;
    }

    TickCountBegin();//(1) Hand Segmentation
    //
    // Segment Hand Region
    //
#ifdef PROCESS_320x240
    cvCopyImage( _HandRegion.GetHandRegion( _pImage320x240, _pGray, _fScreenshot ), _pSkinColorImage );
    IplImage * handRegion = _pSkinColorImage;
#else
    cvCopyImage( _HandRegion.GetHandRegion( _pImage, _pGray, _fScreenshot ), _pSkinColorImage );
    IplImage * handRegion = _pSkinColorImage;
#endif
    TickCountEnd();//(1) Hand Segmentation
    if ( _fScreenshot )
    {
        cvSaveImage( "screenshot_handregion.png", handRegion );
    }

    //
    // Find Fingertip Candidates
    //
    if ( _fFingertipDetected == false )
    {
        _FingerTip.Reset();
    }
    TickCountBegin();//(2) Fingertip Detection
    int nFingertipCandidates = _FingerTip.FindFingerTipCandidatesByCurvature( handRegion, _fScreenshot );
    TickCountEnd();//(2) Fingertip Detection
//    int nFingertipCandidates = _FingerTip.FindFingerTipCandidates( handRegion );
    if ( _fScreenshot )
    {
        // dist transform image
        IplImage * tempImage = cvCreateImage( cvGetSize( _FingerTip._pDistImage ), 8, 1 );
        for ( int i = 0 ; i < _FingerTip._pDistImage->height ; i ++ )
            for ( int j = 0 ; j < _FingerTip._pDistImage->width ; j ++ )
            {
                int value = 4 * cvGetReal2D( _FingerTip._pDistImage, i, j );
                if ( value > 255 ) value = 255;
                cvSetReal2D( tempImage, i, j, value );
            }
        cvSaveImage( "screenshot_distance.png", tempImage );
        cvReleaseImage( &tempImage );

        // contour with max dist point
#ifdef PROCESS_320x240
        tempImage = cvCloneImage( _pImage320x240 );
#else
        tempImage = cvCloneImage( _pImage );
#endif
        cvCircle(
            tempImage,
            cvPoint( _FingerTip._maxDistPoint.x, _FingerTip._maxDistPoint.y ),
            2,
            CV_RGB(255,0,0), -1, 8, 0 );
        cvCircle(
            tempImage,
            cvPoint( _FingerTip._maxDistPoint.x, _FingerTip._maxDistPoint.y ),
            _FingerTip._maxDistValue,
            CV_RGB(255,255,255), 1, 8, 0 );/**/
        CvMemStorage *  pStorage = cvCreateMemStorage(0);
        CvSeq *         pContours;   
        cvFindContours( _FingerTip._pHandImage, pStorage, &pContours, sizeof(CvContour),
            CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,cvPoint(0,0) );
        for ( ; pContours ; pContours = pContours->h_next )
        {
            cvDrawContours( tempImage, pContours, CV_RGB(255,255,0), CV_RGB(0,0,0), 0, 2, 8 );
        }
        cvReleaseMemStorage( &pStorage );
        cvSaveImage( "screenshot_contour.png", tempImage );
        cvReleaseImage( &tempImage );
    }

    //
    // Get Centroid
    //
    _PrevCentroid = _CurrCentroid;
    _CurrCentroid = _FingerTip._maxDistPoint;

//    //
//    // Learn Color Distribution before detecting fingertips
//    //
//    if ( _FingerTip._maxDistPoint.x > _FingerTip._maxDistValue &&
//         _FingerTip._maxDistPoint.x < _FingerTip._pDistImage->width - _FingerTip._maxDistValue &&
//         _FingerTip._maxDistPoint.y > _FingerTip._maxDistValue &&
//         _FingerTip._maxDistPoint.y < _FingerTip._pDistImage->height- _FingerTip._maxDistValue )
//    {
//        cvSetZero( _HandRegion.QueryHandMask() );
//        cvCircle( _HandRegion.QueryHandMask(), _FingerTip._maxDistPoint, 0.7*_FingerTip._maxDistValue, cvScalar(255), -1 );
////        _HandRegion.LearnColor( _pImage, _HandRegion.QueryHandMask() );
//        _HandRegion.LearnColorCrCb( true );
//    }
//    else
//    {
////        _HandRegion.LearnColor( 0, 0 );
//        _HandRegion.LearnColorCrCb( false );
//    }

    //
    // Detect / Track Fingertips
    //
    if ( _fFingertipDetected == false )
    {
        // Not detected yet. Try to detect 5 fingertips.
        TickCountBegin();//(3) Fingertip Tracking
        _fFingertipDetected = _FingertipTracker.FeedFingertipCandidates(
            _pGray,
            nFingertipCandidates,
            _FingerTip._endPoint,
            _FingerTip._endPointDist,
            _CurrCentroid );
        
        cvSetZero( &_FingerRotationMat );
        cvSetZero( &_FingerTranslationMat );
        cvSetZero( &_FingerQuaternionMat );
        TickCountEnd();//(3) Fingertip Tracking
    }
    else
    {
        // Detected.

        // Predict by Kalman filter
        if ( _pKalman )//&& !_fBuildHandModel )
        {
            cvKalmanPredict( _pKalman );

            // Apply the prediction to the fingertip points
/*            _FingertipTracker.PredictFingertips(
                &_IntrinsicMat,
                &_DistortionMat,
                &_FingerRotationMat,
                &_FingerTranslationMat ); /**/
        }

        // Track the fingertips.
/*        _fFingertipDetected = _FingertipTracker.TrackFingertips(
            _pGray,
            nFingertipCandidates,
            _FingerTip._endPoint,
            _FingerTip._endPointDist );
            /**/
        TickCountBegin();//(3) Fingertip Tracking
        _fFingertipDetected = _FingertipTracker.TrackFingertips3(
            nFingertipCandidates,
            _FingerTip._endPoint,
            _FingerTip._endPointDist,
            _PrevCentroid,
            _CurrCentroid );
        TickCountEnd();//(3) Fingertip Tracking

        if ( _fFingertipDetected == false )
        {
            cvSetZero( &_FingerRotationMat );
            cvSetZero( &_FingerTranslationMat );
            cvSetZero( &_FingerQuaternionMat );
        }
    }

    //
    // Learn Color Distribution
    //
    TickCountBegin();//(4) Color Histogram Learning
    if ( _FingertipTracker.QueryTrackingMode() == MODE_FINGERTIP_NONE ||
         _FingertipTracker.QueryTrackingMode() == MODE_FINGERTIP_TRACKING )
    {
        cvSetZero( _HandRegion.QueryHandMask() );
        cvCircle( _HandRegion.QueryHandMask(), _FingerTip._maxDistPoint, 0.7*_FingerTip._maxDistValue, cvScalar(255), -1 );
        _HandRegion.LearnColorCrCb( true );
    }
    TickCountEnd();//(4) Color Histogram Learning

    //
    // Build (or Improve) a Hand Model
    //
    if ( _fBuildHandModel && _fPoseEstimatedByChessboard && _fFingertipDetected )
    {
        _FingertipTracker.CalculateCoordinates(
            _PoseEstimation.QueryProjectionMat() );
    }

    //
    // Pose Estimation from Fingertips
    //
    bool fPrevPoseEstimated = _fPoseEstimatedByFingertips;
    bool fForceOrientation = false;
    _fPoseEstimatedByFingertips = false;
    if ( _fFingertipDetected )
    {
        TickCountBegin();//(5) Pose Estimation
        float rotation_old[3];
        float translation_old[3];
        float quaternion_old[4];
        float center_old[3];
        for ( int i = 0 ; i < 3 ; i ++ )
        {
            rotation_old[i] = _FingerRotation[i];
            translation_old[i] = _FingerTranslation[i];
            quaternion_old[i] = _FingerQuaternion[i];
            center_old[i] = _CameraCenterT[i];
        }
        quaternion_old[3] = _FingerQuaternion[3];
        if ( _fPoseEstimatedByFingertips == false )
        {
            cvSetZero( &_FingerRotationMat );
            cvSetZero( &_FingerTranslationMat );
            cvSetZero( &_FingerQuaternionMat );
        }
        _fPoseEstimatedByFingertips = _FingertipTracker.FindExtrinsicCameraParams(
            &_IntrinsicMat,
            &_DistortionMat,
            &_FingerRotationMat,
            &_FingerTranslationMat );
        //
        // Check Camera Center Range
        //
        cvRodrigues2( &_FingerRotationMat, &_FingerRotation3by3Mat );
        Matrix2Quaternion( _FingerRotation3by3, _FingerQuaternion );
        // check quaternion to be smooth with previous.
        //if ( center_old[0] || center_old[1] || center_old[2] )
        /*{
            for ( int i = 0 ; i < 4 ; i ++ )
            {
                if ( quaternion_old[i] - _FingerQuaternion[i] < -1 )
                {
                    _FingerQuaternion[i] = -2 + _FingerQuaternion[i];
                }
                else if ( quaternion_old[i] - _FingerQuaternion[i] > 1 )
                {
                    _FingerQuaternion[i] = 2 + _FingerQuaternion[i];
                }
            }
            Quaternion2Matrix( _FingerQuaternion, _FingerRotation3by3 );
        }*/
        cvInvert( &_FingerRotation3by3Mat, &_CameraCenterRMat );
        cvMatMul( &_CameraCenterRMat, &_FingerTranslationMat, &_CameraCenterTMat );
        cvScale( &_CameraCenterTMat, &_CameraCenterTMat, -1 );
        float diffQuaternion = 0;
        for ( int i = 0 ; i < 4 ; i ++ )
            diffQuaternion += pow( _FingerQuaternion[i] - quaternion_old[i], 2 );
        diffQuaternion = sqrt( diffQuaternion );
        if ( //abs( _CameraCenterT[2] ) > 2000.0 || _CameraCenterT[2] > 0 ||
             abs( _CameraCenterT[2] ) > 2000.0 ||
             ( _FingertipTracker.QueryFlipOrder() == false && _CameraCenterT[2] > 0 ) ||
             ( _FingertipTracker.QueryFlipOrder() == true  && _CameraCenterT[2] < 0 ) ||
             ( center_old[0] || center_old[1] || center_old[2] ) &&
             //_fValidPose &&
             //diffQuaternion > 1.0 )
             
             sqrt( pow(_CameraCenterT[0] - center_old[0],2)+
                   pow(_CameraCenterT[1] - center_old[1],2)+
                   pow(_CameraCenterT[2] - center_old[2],2) ) > 200 ) /**/
        {
            printf("Invalid Camera (by Fingertip) (%5.2f, %5.2f, %5.2f)\n", _CameraCenterT[0], _CameraCenterT[1], _CameraCenterT[2] );
            _fPoseEstimatedByFingertips = false;
        }
        else
        {
            printf("OK Camera (by Fingertip) (%5.2f, %5.2f, %5.2f) diffQ = %5.2f\n", _CameraCenterT[0], _CameraCenterT[1], _CameraCenterT[2], diffQuaternion );
        }
        printf( "T(%5.2f %5.2f %5.2f) Q(%5.2f %5.2f %5.2f %5.2f)\n",
            _FingerTranslation[0], _FingerTranslation[1], _FingerTranslation[2],
            _FingerQuaternion[0], _FingerQuaternion[1], _FingerQuaternion[2], _FingerQuaternion[3] );

        TickCountEnd();//(5) Pose Estimation
        TickCountBegin();//(6) Kalman Filter

        // when the orientation changes completely (e.g. -0.97 --> 0.98),
        // force the orientation without kalman filtering
        if ( ( center_old[0] || center_old[1] || center_old[2] ) && diffQuaternion > 1.0 )
        {
            fForceOrientation = true;
            for ( int i = 0 ; i < 4 ; i ++ )
            {
                cvSetReal1D( _pKalman->state_pre, i+3, _FingerQuaternion[i] );
                cvSetReal1D( _pKalman->state_pre, i+10, 0 );
            }
        }

        if ( _fPoseEstimatedByFingertips == false )//||
//             abs( _CameraCenterT[2] ) > 2000.0 ||
//             ( _FingertipTracker.QueryFlipOrder() == false && _CameraCenterT[2] > 0 ) ||
//             ( _FingertipTracker.QueryFlipOrder() == true  && _CameraCenterT[2] < 0 ) )//&& fPrevPoseEstimated == true )
        {
            //
            // When the pose estimation is unreliable, use predicted or old pose.
            //
/*            if ( _pKalman )
            {
                for ( int i = 0 ; i < 3 ; i ++ )
                {
                    _FingerTranslation[i] = cvGetReal1D( _pKalman->state_pre, i   );
                    _FingerRotation[i]    = cvGetReal1D( _pKalman->state_pre, i+3 );
                }
            }
            else/**/
            {
                for ( int i = 0 ; i < 3 ; i ++ )
                {
                    _FingerRotation[i] = rotation_old[i];
                    _FingerTranslation[i] = translation_old[i];
                    _FingerQuaternion[i] = quaternion_old[i];
                }
                _FingerQuaternion[3] = quaternion_old[3];
                /**/
                printf("forced to be old pose.\n");
            }
            //_fPoseEstimatedByFingertips = fPrevPoseEstimated;
        }

        //if ( !_fBuildHandModel )
        else
        {
            // Init Kalman filter if necessary
            if ( _pKalman == 0 )
            {
                _pKalman = cvCreateKalman( 14, 7, 0 );
                for ( int i = 0 ; i < 7 ; i ++ )
                    cvSetReal2D( _pKalmanMat_A, i, i+7, 1 );
                cvCopy( _pKalmanMat_A, _pKalman->transition_matrix );
                cvSetIdentity( _pKalman->measurement_matrix, cvRealScalar(1) );
                // noise covariances
                cvSetZero( _pKalman->process_noise_cov );
                cvSetZero( _pKalman->measurement_noise_cov );
                for ( int i = 0 ; i < 3 ; i ++ )
                {
                    // translation part
                    cvSetReal2D( _pKalman->process_noise_cov, i, i, 10 );
                    cvSetReal2D( _pKalman->process_noise_cov, i+7, i+7, 10 );
                    cvSetReal2D( _pKalman->measurement_noise_cov, i, i, 30 );
                }
                for ( int i = 0 ; i < 4 ; i ++ )
                {
                    // rotation part
                    cvSetReal2D( _pKalman->process_noise_cov, i+3, i+3, 0.2 );
                    cvSetReal2D( _pKalman->process_noise_cov, i+10, i+10, 0.2 );
                    cvSetReal2D( _pKalman->measurement_noise_cov, i+3, i+3, 0.5 );
                }
                cvSetIdentity( _pKalman->error_cov_post, cvRealScalar(1) );
                cvSetZero( _pKalman->state_post );
                for ( int i = 0 ; i < 3 ; i ++ )
                {
                    cvSetReal1D( _pKalman->state_post, i,   _FingerTranslation[i] );
                }
                for ( int i = 0 ; i < 4 ; i ++ )
                {
                    cvSetReal1D( _pKalman->state_post, i+3, _FingerQuaternion[i] );
                }
            }
            else
            {
                // Kalman correct
                float measurement[7];
                CvMat measurementMat = cvMat( 7, 1, CV_32FC1, measurement );
                for ( int i = 0 ; i < 3 ; i ++ )
                {
                    measurement[i]   = _FingerTranslation[i];
                }
                for ( int i = 0 ; i < 4 ; i ++ )
                {
                    measurement[i+3] = _FingerQuaternion[i];
                }
                // update A's delta T
                for ( int i = 0 ; i < 7 ; i ++ )
                    cvSetReal2D( _pKalmanMat_A, i, i+7, _deltaT );
                cvCopy( _pKalmanMat_A, _pKalman->transition_matrix );

                // correct kalman filter state
                cvKalmanCorrect( _pKalman, &measurementMat );
/*
                _Rng.disttype = CV_RAND_NORMAL;
                float rand[4];
                CvMat randMat = cvMat( 3, 1, CV_32FC1, rand );
                for ( int i = 0 ; i < 14 ; i +=7 )
                {
                    cvRandSetRange( &_Rng, 0, sqrt(_pKalman->process_noise_cov->data.fl[i]), 0 );
                    cvRand( &_Rng, &randMat );
                    cvSetReal1D( _pKalmanMat_W, i  , rand[0] );
                    cvSetReal1D( _pKalmanMat_W, i+1, rand[1] );
                    cvSetReal1D( _pKalmanMat_W, i+2, rand[2] );
                }
                randMat = cvMat( 4, 1, CV_32FC1, rand );
                for ( int i = 0 ; i < 14 ; i +=7 )
                {
                    cvRandSetRange( &_Rng, 0, sqrt(_pKalman->process_noise_cov->data.fl[i+3]), 0 );
                    cvRand( &_Rng, &randMat );
                    cvSetReal1D( _pKalmanMat_W, i+3, rand[0] );
                    cvSetReal1D( _pKalmanMat_W, i+4, rand[1] );
                    cvSetReal1D( _pKalmanMat_W, i+5, rand[2] );
                    cvSetReal1D( _pKalmanMat_W, i+6, rand[3] );
                }
                cvMatMulAdd(
                    _pKalman->transition_matrix,
                    _pKalman->state_post,
                    _pKalmanMat_W,
                    _pKalman->state_post );

                    */
            }

            // Copy the corrected state
            for ( int i = 0 ; i < 3 ; i ++ )
            {
                _FingerTranslation[i] = cvGetReal1D( _pKalman->state_post, i   );
            }
            for ( int i = 0 ; i < 4 ; i ++ )
            {
                _FingerQuaternion[i]  = cvGetReal1D( _pKalman->state_post, i+3 );
            }/**/
        }
        TickCountEnd();//(6) Kalman Filter
    }
    else
    {
        _fPoseEstimatedByFingertips = false;

        // Reset Kalman filter
        if ( _pKalman )
        {
            cvReleaseKalman( &_pKalman );
            _pKalman = 0;
        }
        
        TickCountDummy();//(5) Pose Estimation
        TickCountDummy();//(6) Kalman Filter
    }

    _fValidPose = _fPoseEstimatedByFingertips || (_fFingertipDetected && fPrevPoseEstimated);
    printf( "pose: %s\n", _fValidPose?"valid":"invalid" );

    //
    // Compute Camera Center
    //
    //cvRodrigues2( &_FingerRotationMat, &_FingerRotation3by3Mat );
    printf( "T(%5.2f %5.2f %5.2f) Q(%5.2f %5.2f %5.2f %5.2f)\n",
        _FingerTranslation[0], _FingerTranslation[1], _FingerTranslation[2],
        _FingerQuaternion[0], _FingerQuaternion[1], _FingerQuaternion[2], _FingerQuaternion[3] );
    Quaternion2Matrix( _FingerQuaternion, _FingerRotation3by3 );
    cvRodrigues2( &_FingerRotation3by3Mat, &_FingerRotationMat );
    cvInvert( &_FingerRotation3by3Mat, &_CameraCenterRMat );
    cvMatMul( &_CameraCenterRMat, &_FingerTranslationMat, &_CameraCenterTMat );
    cvScale( &_CameraCenterTMat, &_CameraCenterTMat, -1 );
    printf( "Camera(by Fingertips) = (%5.2f, %5.2f, %5.2f)\n", _CameraCenterT[0], _CameraCenterT[1], _CameraCenterT[2]);
/*
    printf( "T(%5.2f %5.2f %5.2f) Q(%5.2f %5.2f %5.2f %5.2f)\n",
        _FingerTranslation[0], _FingerTranslation[1], _FingerTranslation[2],
        _FingerQuaternion[0], _FingerQuaternion[1], _FingerQuaternion[2], _FingerQuaternion[3] );
    cvRodrigues2( &_ChessRotationMat, &_FingerRotation3by3Mat );
    float r_[3][3];
    CvMat r_inv = cvMat(3, 3, CV_32FC1, r_);
    cvInvert( &_FingerRotation3by3Mat, &r_inv );
    cvMatMul( &r_inv, &_ChessTranslationMat, &_CameraCenterTMat );
    cvScale( &_CameraCenterTMat, &_CameraCenterTMat, -1 );
    printf( "Camera(by Fingertips) = (%5.2f, %5.2f, %5.2f)\n", _CameraCenterT[0], _CameraCenterT[1], _CameraCenterT[2]);
*/

    //
    // Compute Difference btw Chessboard and Fingertips
    //
    if ( _fPoseEstimatedByChessboard && _fValidPose )//_fPoseEstimatedByFingertips )
    {
        for ( int i = 0 ; i < 3 ; i ++ )
        {
            _CenterTDiff[i] = abs( chess_center[i] - _CameraCenterT[i] );
            fprintf( _fpDiff, "%.5f\t", _CenterTDiff[i] );
        }

        float error = 0;

        //
        // Compute RMS reprojection error for chessboard
        //
        error = _PoseEstimation.ComputeReprojectionError( _pImage, &_ChessRotationMat, &_ChessTranslationMat );
        if ( error >= 0 )
        {
            fprintf( _fpDiff, "%.5f\t", error );
        }
        else
        {
            fprintf( _fpDiff, " \t" );
        }

        //
        // Compute RMS reprojection error
        //
        error = _PoseEstimation.ComputeReprojectionError( _pImage, &_FingerRotationMat, &_FingerTranslationMat, _fScreenshot );
        if ( error >= 0 )
        {
            fprintf( _fpDiff, "%.5f\t", error );
        }
        else
        {
            fprintf( _fpDiff, " \t" );
        }

        //
        // Compute RMS reprojection error for ARTag
        //
        if ( _artag_chess_found )
        {
            error = _PoseEstimation.ComputeReprojectErroForARTag( _pImage, _artag_chess_pt, _fScreenshot );
            fprintf( _fpDiff, "%.5f\t", error );
        }
        else
        {
            fprintf( _fpDiff, " \t" );
        }

        fprintf( _fpDiff, "\n" );
    }

}

IplImage * FingertipPoseEstimation::OnDisplay( IplImage * image )
{
    if ( !image )
    {
        image = _pImage;
    }

    //
    // Debug Images
    //

    // Contour
    /*
    CvMemStorage *  pStorage = cvCreateMemStorage(0);
    CvSeq *         pContours;   
    cvFindContours( _FingerTip._pHandImage, pStorage, &pContours, sizeof(CvContour),
        CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,cvPoint(0,0) );
#ifdef PROCESS_320x240
    cvSetZero( image320x240 );
    for ( ; pContours ; pContours = pContours->h_next )
    {
        cvDrawContours( image320x240, pContours, CV_RGB(255,255,0), CV_RGB(0,0,255), 0, 1, 8 );
    }
    cvPyrUp( image320x240, image );
#else
    //cvSetZero( image );
    for ( ; pContours ; pContours = pContours->h_next )
    {
        cvDrawContours( image, pContours, CV_RGB(255,255,0), CV_RGB(0,0,255), 0, 1, 8 );
    }
#endif
    cvReleaseMemStorage( &pStorage );
    /**/

    // Skin-color segmentation
    /*
#ifdef PROCESS_320x240
    cvMerge( _HandRegion.QueryHandRegion(),
        _HandRegion.QueryHandRegion(),
        _HandRegion.QueryHandRegion(),
        0,
        image320x240 );
    cvPyrUp( image320x240, image );
#else
    cvMerge( _HandRegion.QueryHandRegion(),
        _HandRegion.QueryHandRegion(),
        _HandRegion.QueryHandRegion(),
        0,
        image );
#endif
    /**/

    // Single Connected Component
    /*
#ifdef PROCESS_320x240
    cvMerge( _FingerTip._pHandImage,
        _FingerTip._pHandImage,
        _FingerTip._pHandImage,
        0,
        image320x240 );
    cvPyrUp( image320x240, image );
#else
    cvMerge( _FingerTip._pHandImage,
        _FingerTip._pHandImage,
        _FingerTip._pHandImage,
        0,
        image );
#endif
    /**/

//    cvCopy( _HandRegion.imageYCrCb, image );

    // Distance Transform
    /*
#ifdef PROCESS_320x240
    for ( int i = 0 ; i < 320 ; i ++ )
        for ( int j = 0 ; j < 240 ; j ++ )
        {
            int value = 4 * cvGetReal2D( _FingerTip._pDistImage, j, i );
            if ( value > 255 ) value = 255;
            cvSet2D( image320x240, j, i, CV_RGB(value, value, value));
        }
    cvPyrUp( image320x240, image );
#else
    for ( int i = 0 ; i < image->width ; i ++ )
        for ( int j = 0 ; j < image->height ; j ++ )
        {
            int value = 4 * cvGetReal2D( _FingerTip._pDistImage, j, i );
            if ( value > 255 ) value = 255;
            cvSet2D( image, j, i, CV_RGB(value, value, value));
        }
#endif
/**/

/*    //
    // Background Segmentation
    //
    if ( _ColorDistribution.QueryMask() )
    {
        cvSetImageCOI( image, 1 );
        cvCopyImage( _ColorDistribution.QueryMask(), image );
        cvSetImageCOI( image, 0 );
    }/**/

    //
    // Fingertips ( Candidates )
    //
    static CvScalar color[] = {
        {{64,64,255}},   // red
        {{0,128,255}},  // orange
        {{0,255,255}},  // yellow
        {{0,192,0}},  // green
        {{255,0,0}}     // blue
    };

    for ( int i = 0 ; i < NUM_FINGERTIP; i ++ )
    {
        CvPoint2D32f * pt = _FingertipTracker.QueryFingertip( i );
        if ( pt )
        {
#ifdef PROCESS_320x240
            cvCircle(
                image,
                cvPoint( pt->x * 2, pt->y * 2 ),
                _FingertipTracker.QueryFingertipDist( i ) * 2,
                _fFingertipDetected ? color[i] : CV_RGB(255,0,0),
                -1,
                8,
                0 );
#else
            cvCircle(
                image,
                cvPointFrom32f( *pt ),
                _FingertipTracker.QueryFingertipDist( i ),
                _fFingertipDetected ? color[i] : CV_RGB(255,0,0),
                -1,
                8,
                0 );
#endif
        }
    }/**/

    for ( int i = 0 ; i < _FingerTip._numEndPoint ; i ++ )
    {
        /*
        cvCircle(
            image,
            _FingerTip.QueryEndPoint( i ),
            _FingerTip.QueryEndPointDist( i ),
            CV_RGB(0,255,0),
            1,8,0 );/**/
        
#ifdef PROCESS_320x240
        CvPoint center = cvPoint( _FingerTip._Ellipse[i].center.x * 2, _FingerTip._Ellipse[i].center.y * 2 );
        CvSize size = cvSize( _FingerTip._Ellipse[i].size.width * 1, _FingerTip._Ellipse[i].size.height * 1 );
#else
        CvPoint center = cvPointFrom32f( _FingerTip._Ellipse[i].center );
        CvSize size = cvSize( _FingerTip._Ellipse[i].size.width * 0.5, _FingerTip._Ellipse[i].size.height * 0.5 );
#endif
        if ( size.width > 0 && size.height > 0 &&
             size.width < image->width && size.height < image->height )
        {
            cvEllipse(
                image,
                center,
                size,
                - _FingerTip._Ellipse[i].angle,
                0,
                360,
                CV_RGB(0,255,0),
                1,
                CV_AA, 0 );
        }/**/
    }

    //
    // Max Distance Point + Region of Interest
    //
    
#ifdef PROCESS_320x240
    cvCircle(
        image,
        cvPoint( _FingerTip._maxDistPoint.x * 2, _FingerTip._maxDistPoint.y * 2 ),
        _FingerTip._maxDistValue * 0.7 * 2,
        CV_RGB(255,255,0), 1, 8, 0 );
    int i1 = _FingerTip._maxDistPoint.y - 4 * _FingerTip._maxDistValue;
    int i2 = _FingerTip._maxDistPoint.y + 4 * _FingerTip._maxDistValue;
    int j1 = _FingerTip._maxDistPoint.x - 4 * _FingerTip._maxDistValue;
    int j2 = _FingerTip._maxDistPoint.x + 4 * _FingerTip._maxDistValue;
    //cvRectangle( image, cvPoint(j1*2, i1*2), cvPoint(j2*2, i2*2), CV_RGB(0,255,0), 1 );
#else
    cvCircle( image, _FingerTip._maxDistPoint, _FingerTip._maxDistValue * 0.7, CV_RGB(255,255,0), 1, 8, 0 );
    int i1 = _FingerTip._maxDistPoint.y - 4 * _FingerTip._maxDistValue;
    int i2 = _FingerTip._maxDistPoint.y + 4 * _FingerTip._maxDistValue;
    int j1 = _FingerTip._maxDistPoint.x - 4 * _FingerTip._maxDistValue;
    int j2 = _FingerTip._maxDistPoint.x + 4 * _FingerTip._maxDistValue;
    //cvRectangle( image, cvPoint(j1, i1), cvPoint(j2, i2), CV_RGB(0,255,0), 1 );
#endif
    /**/

    //
    // Hand Contour
    //
    //_HandRegion.DrawContour( image, _FingerTip._maxDistPoint, _FingerTip._pDistImage );

    //
    // Difference btw Chessboard and Fingertips
    //
    /*
    if ( _fPoseEstimatedByChessboard && _fPoseEstimatedByFingertips )
    {
        cvRectangle( image, cvPoint( image->width - 60, 0 ), cvPoint( image->width - 40, _CenterTDiff[0] ), CV_RGB(255,0,0), -1, 8, 0 );
        cvRectangle( image, cvPoint( image->width - 40, 0 ), cvPoint( image->width - 20, _CenterTDiff[1] ), CV_RGB(0,255,0), -1, 8, 0 );
        cvRectangle( image, cvPoint( image->width - 20, 0 ), cvPoint( image->width     , _CenterTDiff[2] ), CV_RGB(0,0,255), -1, 8, 0 );
    }/**/

    // FPS
/*    char str[255];
    static double timestamp;
    double pasttime = timestamp;
    timestamp = (double)clock()/CLOCKS_PER_SEC;
    sprintf(str, "FPS=%.1f", 1.0/(timestamp - pasttime));
    cvPutText(image, str, cvPoint(5, 5), &font, CV_RGB(255, 255, 255));
    fprintf( _fpTime, "%.5f\t", 1.0/(timestamp - pasttime));
/**/
    // Save screens
    if ( _fScreenshot )
    {
        cvSaveImage( "screenhot_final.png", image );
        _fScreenshot = false;
    }

    return image;
}

bool FingertipPoseEstimation::QueryValidPose()
{
    return _fValidPose;
}

CvMat FingertipPoseEstimation::QueryRotationMat()
{
    return _FingerRotationMat;
}

CvMat FingertipPoseEstimation::QueryTranslationMat()
{
    return _FingerTranslationMat;
}

float * FingertipPoseEstimation::QueryModelViewMat()
{
/*    // OpenGL ModelView Matrix of World-2-Camera
    for ( int i = 0 ; i < 3 ; i ++ )
    {
        for ( int j = 0 ; j < 3 ; j ++ )
            _ModelView[j*4+i] = _FingerRotation3by3[i][j];
        //_ModelView[8+i] *= -1;
        _ModelView[12+i] = _FingerTranslation[i];
    }
    //_ModelView[14] *= -1;
    _ModelView[3] = _ModelView[7] = _ModelView[11] = 0;
    _ModelView[15] = 1; /**/

    return _ModelView;
}

bool FingertipPoseEstimation::LoadFingertipCoordinates( char * filename )
{
    return _FingertipTracker.LoadFingertipCoordinates( filename );
}

bool FingertipPoseEstimation::SaveFingertipCoordinates( char * filename )
{
    return _FingertipTracker.SaveFingertipCoordinates( filename );
}

bool FingertipPoseEstimation::ToggleChessboard()
{
    _fFindChessboard = !_fFindChessboard;

    return _fFindChessboard;
}

bool FingertipPoseEstimation::ToggleBuildHandModel()
{
    //
    // Translate to center when the building process is done.
    //
    if ( _fBuildHandModel == true )
    {
#ifdef TRANSLATE_COORDINATE
        _FingertipTracker.CentralizeCoordinates();
#endif
    }

    // Toggle
    _fBuildHandModel = !_fBuildHandModel;

    // Reset when starting
    if ( _fBuildHandModel == true )
    {
        _FingertipTracker.ResetCoordinates();
    }

    return _fBuildHandModel;
}

void FingertipPoseEstimation::SetOpenGLFrustrum()
{
    if ( !_pImage )
        return;

    //set viewing frustrum to match camera FOV (ref: ARTag's 3d_augmentations.cpp)
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    double camera_fx = _CameraIntrinsic[0][0];
    double camera_fy = _CameraIntrinsic[1][1];
    double camera_ox = _CameraIntrinsic[0][2];
    double camera_oy = _CameraIntrinsic[1][2];
    double camera_opengl_dRight = (double)(_pImage->width-camera_ox)/(double)(camera_fx);
    double camera_opengl_dLeft = -camera_ox/camera_fx;
    double camera_opengl_dTop = (double)(_pImage->height-camera_oy)/(double)(camera_fy);
    double camera_opengl_dBottom = -camera_oy/camera_fy;
    glFrustum(camera_opengl_dLeft, camera_opengl_dRight, camera_opengl_dBottom, camera_opengl_dTop, 1.0, 102500.0);
    camera_opengl_dLeft *= 10240;
    camera_opengl_dRight *= 10240;
    camera_opengl_dBottom *= 10240;
    camera_opengl_dTop *= 10240;

    // convert image R and B channel
    cvConvertImage( _pImage, _pImage, CV_CVTIMG_SWAP_RB );

    // draw the camera frame
    glBindTexture(GL_TEXTURE_2D, _nCameraTexID);
    glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, _pImage->width, _pImage->height, GL_RGB, GL_UNSIGNED_BYTE, _pImage->imageData );

    glClearColor( 0, 0, 0, 0 );

    // draw a quad for the background video
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_LIGHTING);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, _nCameraTexID);

    glBegin(GL_QUADS);
    glColor3f(1.0f, 1.0f, 1.0f);
    //draw camera texture, set with offset to aim only at cam_width x cam_height upper left bit
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(camera_opengl_dLeft, camera_opengl_dBottom, -10240);
    glTexCoord2f(0.0f, (float)_pImage->height/1024.0);          
    glVertex3f(camera_opengl_dLeft, camera_opengl_dTop, -10240);
    glTexCoord2f((float)_pImage->width/1024.0, (float)_pImage->height/1024.0);  
    glVertex3f(camera_opengl_dRight, camera_opengl_dTop, -10240);
    glTexCoord2f((float)_pImage->width/1024.0, 0.0f);          
    glVertex3f(camera_opengl_dRight, camera_opengl_dBottom, -10240);
    glEnd();

    glDisable( GL_TEXTURE_2D );
    glEnable( GL_LIGHTING );
}

void FingertipPoseEstimation::SetOpenGLModelView()
{
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    gluLookAt(
          _CameraCenterT[0],
          _CameraCenterT[1],
        - _CameraCenterT[2],
          _CameraCenterT[0] + _FingerRotation3by3[2][0],
          _CameraCenterT[1] + _FingerRotation3by3[2][1],
        -(_CameraCenterT[2] + _FingerRotation3by3[2][2]),
          _FingerRotation3by3[1][0],
          _FingerRotation3by3[1][1],
        - _FingerRotation3by3[1][2] );
    glGetFloatv( GL_MODELVIEW_MATRIX, _ModelView );

}

void FingertipPoseEstimation::RenderCaptureImage()
{
}

float FingertipPoseEstimation::QueryFx()
{
    return _CameraIntrinsic[0][0];
}

float FingertipPoseEstimation::QueryFy()
{
    return _CameraIntrinsic[1][1];
}

void FingertipPoseEstimation::SetARTagChessboardFound( bool fFound, float chess_pt[][2] )
{
    _artag_chess_found = fFound;
    if ( _artag_chess_found && chess_pt )
    {
        for ( int i = 0 ; i < NUM_CHESS_POINT ; i ++ )
        {
            _artag_chess_pt[i][0] = chess_pt[i][0];
            _artag_chess_pt[i][1] = chess_pt[i][1];
        }
    }
}

float ** FingertipPoseEstimation::QueryARTagChessboard()
{
    return (float **)_artag_chess_pt;
}

float FingertipPoseEstimation::QueryDeltaT()
{
    return _deltaT;
}

int FingertipPoseEstimation::QueryCentroidX()
{
    return _FingerTip._maxDistPoint.x;
}

int FingertipPoseEstimation::QueryCentroidY()
{
    return _FingerTip._maxDistPoint.y;
}

void FingertipPoseEstimation::PrintScreenshot()
{
    _fScreenshot = true;
}
