#include <HandyAR/FingertipTracker.h>

FingertipTracker::FingertipTracker(void)
{
    _nMode = MODE_FINGERTIP_NONE;

    _nPoints = 0;

    _fpCoordinates = 0;
    _numCoordinateSamples = 0;
//    cvRandInit( &_Rng, 0, 1, -1, CV_RAND_UNI );

    _nFingertips = 0;
    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
        _pFingertipTemplate[i] = 0;

/*    // Kalman filters
    for ( int i = 0 ; i < NUM_MAX_CANDIDATES ; i ++ )
    {
        _pKalman[i] = 0;
    }
    _pKalmanMat_A = cvCreateMat( 4, 4, CV_32FC1 );  //  A = [ 1 0 1 0 ]
    cvSetIdentity( _pKalmanMat_A );                 //      | 0 1 0 1 |
    cvSetReal2D( _pKalmanMat_A, 0, 2, 1 );          //      | 0 0 1 0 |
    cvSetReal2D( _pKalmanMat_A, 1, 3, 1 );          //      [ 0 0 0 1 ]

    _pKalmanMat_G = cvCreateMat( 4, 2, CV_32FC1 );  //  G = [ 0 0 ]
    cvSetZero( _pKalmanMat_G );                     //      | 0 0 |
    cvSetReal2D( _pKalmanMat_G, 2, 0, 1 );          //      | 1 0 |
    cvSetReal2D( _pKalmanMat_G, 3, 1, 1 );          //      [ 0 1 ]

    _pKalmanMat_H = cvCreateMat( 2, 4, CV_32FC1 );  //  H = [ 1 0 0 0 ]
    cvSetZero( _pKalmanMat_H );                     //      [ 0 1 0 0 ]
    cvSetReal2D( _pKalmanMat_H, 0, 0, 1 );
    cvSetReal2D( _pKalmanMat_H, 1, 1, 1 );

    _pKalmanMat_W = cvCreateMat( 4, 1, CV_32FC1 );  //  W
*/

    _fFingertipCoordinates = false;
    _fInitialFingertip = false;

}

FingertipTracker::~FingertipTracker(void)
{
/*    for ( int i = 0 ; i < NUM_MAX_CANDIDATES ; i ++ )
    {
        if ( _pKalman[i] )
            cvReleaseKalman( &(_pKalman[i]) );
    } */

    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
    {
        if ( _pFingertipTemplate[i] )
            cvReleaseImage( &_pFingertipTemplate[i] );
    }

    if ( _fpCoordinates )
    {
        fclose( _fpCoordinates );
        _fpCoordinates = 0;
    }
}

bool FingertipTracker::LoadFingertipCoordinates( char * filename )
{
    //
    // Load Fingertip Coordinates from the file
    //
    FILE * fp = fopen( filename, "rt" );

    if ( !fp )
        return false;

    float x, y, z;
    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
    {
        fscanf( fp, "%f %f %f", &x, &y, &z );
        _fingertipCoordinates[i][0] = x;
        _fingertipCoordinates[i][1] = y;
        _fingertipCoordinates[i][2] = z;
    }

    fclose( fp );

    _fFingertipCoordinates = true;

    return _fFingertipCoordinates;
}

bool FingertipTracker::SaveFingertipCoordinates( char * filename )
{
    if ( _fFingertipCoordinates == false )
        return false;

    //
    // Save Fingertip Coordinates from the file
    //
    FILE * fp = fopen( filename, "wt" );

    if ( !fp )
        return false;

    float x, y, z;
    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
    {
        x = _fingertipCoordinates[i][0];
        y = _fingertipCoordinates[i][1];
        z = _fingertipCoordinates[i][2];
        fprintf( fp, "%f %f %f\n", x, y, z );
    }

    fclose( fp );

    return true;
}

void FingertipTracker::MatchCorrespondencesByNearestNeighbor( int nPoints, CvPoint * points, float * distValue )
{
    bool * fMatched = (bool *)malloc( nPoints * sizeof(bool) );
    memset( fMatched, 0, nPoints * sizeof(bool) );

    for ( int i = 0 ; i < _nPoints ; i ++ )
    {
        //
        // Find the closest point of each candidate point
        //
        float minDist = THRESHOLD_CLOSEST_DIST;
        int   minJ = -1;
        for ( int j = 0 ; j < nPoints ; j ++ )
        {
            float dist = sqrt(
                pow( (float)points[j].x - _candidatePoints[i].x, 2 ) +
                pow( (float)points[j].y - _candidatePoints[i].y, 2 ) );
            if ( !fMatched[j] && dist < minDist )
            {
                minDist = dist;
                minJ = j;
            }
        }

        if ( minJ >= 0 )
        {
            //
            // If exist update the score and position
            //
            fMatched[minJ] = true;

            _candidateVelocity[i][0] = points[minJ].x - _candidatePoints[i].x;
            _candidateVelocity[i][1] = points[minJ].y - _candidatePoints[i].y;

            _candidatePoints[i].x = points[minJ].x;
            _candidatePoints[i].y = points[minJ].y;

            _candidateDist[i] = distValue[minJ];

            _candidateScore[i] ++;
            _candidateLost [i] = 0;

        }
    }

    for ( int i = 0 ; i < nPoints ; i ++ )
    {
        if ( _nPoints < NUM_MAX_CANDIDATES && fMatched[i] == false )
        {
            //
            // If there is no close point, add a new candidate
            //
            _candidatePoints[_nPoints].x = points[i].x;
            _candidatePoints[_nPoints].y = points[i].y;
            _candidateScore [_nPoints]   = 0;
            _candidateAge   [_nPoints]   = 0;
            _candidateLost  [_nPoints]   = 0;
            _candidateDist  [_nPoints]   = distValue[i];

            _nPoints ++;
        }
    }

    //
    // Release Memory
    //
    free( fMatched );

}

void FingertipTracker::Reset()
{
    _nMode = MODE_FINGERTIP_NONE;
    _nPoints = 0;
    _fDetected = false;
    _nFingertips = 0;
}

bool FingertipTracker::FeedFingertipCandidates(
    IplImage *  image,
    int         nPoints,
    CvPoint *   points,
    float *     distValue,
    CvPoint     currCentroid
)
{
    //
    // Basically increase Lost value. When it is found, it will decrease the value
    //
    for ( int i = 0 ; i < _nPoints ; i ++ )
    {
        _candidateLost[i] ++;
    }

    //
    // Kalman predict
    //
/*    for ( int i = 0 ; i < _nPoints ; i ++ )
    {
        cvKalmanPredict( _pKalman[i] );
    }
/**/

    //
    // Predict by matching Fingertip Templates
    //
    //MatchFingertipTemplate( image );

    //
    // Match the closest point
    //
    MatchCorrespondencesByNearestNeighbor( nPoints, points, distValue );

    //
    // And later, if fingertips are being tracked, we use minimizing net-sum distance method.
    //

/*
    int minJ [NUM_MAX_CANDIDATES] = {-1};
    int minJ2[NUM_MAX_CANDIDATES] = {-1};
    float minDist [NUM_MAX_CANDIDATES] = {THRESHOLD_CLOSEST_DIST};
    float minDist2[NUM_MAX_CANDIDATES] = {THRESHOLD_CLOSEST_DIST};
    for ( int i = 0 ; i < _nPoints ; i ++ )
    {
        //
        // Find the 1st and 2nd closest new points for each candidate point
        //
        for ( int j = 0 ; j < nPoints ; j ++ )
        {
            float tempdist = sqrt(
                pow( (float)points[j].x - _candidatePoints[i].x, 2 ) +
                pow( (float)points[j].y - _candidatePoints[i].y, 2 ) );
            if ( tempdist < minDist[i] )
            {
                minDist2[i] = minDist[i];
                minJ2[i] = minJ[i];
                minDist[i] = tempdist;
                minJ[i] = j;
            }
            else if ( tempdist < minDist2[i] )
            {
                minDist2[i] = tempdist;
                minJ2[i] = j;
            }
        }
    }
    int * nMatched = (int *)malloc( nPoints * sizeof(int) );
    int * nMatchIndex = (int *)malloc( nPoints * sizeof(int) );
    memset( nMatched, 0, nPoints*sizeof(int) );
    memset( nMatchIndex, -1, nPoints*sizeof(int) );
    for ( int i = 0 ; i < _nPoints ; i ++ )
    {
        if ( minJ[i] < 0 ) continue;
        if ( nMatched[ minJ[i] ] == 0 )
        {
            // New match
            nMatchIndex[ minJ[i] ] = i;
            nMatched[ minJ[i] ] = 1;
        }
        else
        {
            // Another match exist. Compare distance
            if ( minJ[ nMatchIndex[ minJ[i] ] ] == minJ[i] )
            {
                if ( minDist[ nMatchIndex[ minJ[i] ] ] >
                     minDist[ i ] )
                {
                    nMatchIndex[ minJ[i] ] = i;
                }
            } else if ( minJ2[ nMatchIndex[ minJ[i] ] ] == minJ[i] )
            {
                if ( minDist2[ nMatchIndex[ minJ[i] ] ] >
                     minDist [ i ] )
                {
                    nMatchIndex[ minJ[i] ] = i;
                }
            }
        }
    }

    //
    // Assign Correspondences
    //
    for ( int i = 0 ; i < nPoints ; i ++ )
    {
        int minJ = nMatchIndex[i];
        if ( minJ >= 0 )
        {
            //
            // for verbose, draw a line btw correspondences
            //
            cvDrawLine( image, points[i], cvPointFrom32f(_candidatePoints[minJ]), CV_RGB(0,255,0), 1,8,0);
            //
            // If exist, update the score and position
            //
            _candidateVelocity[minJ][0] = points[i].x - _candidatePoints[minJ].x;
            _candidateVelocity[minJ][1] = points[i].y - _candidatePoints[minJ].y;

            _candidatePoints[minJ].x = points[i].x;
            _candidatePoints[minJ].y = points[i].y;

            _candidateDist[minJ] = dist[i];

            _candidateScore[minJ] ++;
            _candidateLost [minJ] = 0;

            // Kalman correct
            float pt[2];
            pt[0] = _candidatePoints[minJ].x;
            pt[1] = _candidatePoints[minJ].y;
            CvMat measurementMat = cvMat( 2, 1, CV_32FC1, pt ); // measurement (x,y)
            cvKalmanCorrect( _pKalman[minJ], &measurementMat );

            _Rng.disttype = CV_RAND_NORMAL;
            cvRandSetRange( &_Rng, 0, sqrt(_pKalman[minJ]->process_noise_cov->data.fl[0]), 0 );
            cvRand( &_Rng, _pKalmanMat_W );
            cvMatMulAdd(
                _pKalman[minJ]->transition_matrix,
                _pKalman[minJ]->state_post,
                _pKalmanMat_W,
                _pKalman[minJ]->state_post );

        }
        else if ( _nPoints < NUM_MAX_CANDIDATES )
        {
            //
            // If there is no close point, add a new candidate
            //
            _candidatePoints[_nPoints].x = points[i].x;
            _candidatePoints[_nPoints].y = points[i].y;
            _candidateScore [_nPoints]   = 0;
            _candidateAge   [_nPoints]   = 0;
            _candidateLost  [_nPoints]   = 0;

            // Init Kalman filter
            _pKalman[_nPoints] = cvCreateKalman( 4, 2, 0 );
            cvCopy( _pKalmanMat_A, _pKalman[_nPoints]->transition_matrix );
            cvSetIdentity( _pKalman[_nPoints]->measurement_matrix, cvRealScalar(1) );
            cvSetIdentity( _pKalman[_nPoints]->process_noise_cov, cvRealScalar(1e-5) );
            cvSetIdentity( _pKalman[_nPoints]->measurement_noise_cov, cvRealScalar(1e-1) );
            cvSetIdentity( _pKalman[_nPoints]->error_cov_post, cvRealScalar(1) );
            cvSetReal1D( _pKalman[_nPoints]->state_post, 0, points[i].x );
            cvSetReal1D( _pKalman[_nPoints]->state_post, 1, points[i].y );
            cvSetReal1D( _pKalman[_nPoints]->state_post, 2, 0 );
            cvSetReal1D( _pKalman[_nPoints]->state_post, 3, 0 );

            _nPoints ++;
        }
    }

    // Release memory
    free( nMatched );
    free( nMatchIndex );

/*
    for ( int i = 0 ; i < nPoints ; i ++ )
    {
        //
        // Find the closest candidate point for each new point
        //
        float minDist = THRESHOLD_CLOSEST_DIST;
        int   minJ = -1;
        for ( int j = 0 ; j < _nPoints ; j ++ )
        {
            float dist = sqrt(
                pow( (float)points[i].x - _candidatePoints[j].x, 2 ) +
                pow( (float)points[i].y - _candidatePoints[j].y, 2 ) );
            //float dist = sqrt(
            //    pow( (float)points[i].x - cvGetReal1D( _pKalman[j]->state_pre, 0 ), 2 ) +
            //    pow( (float)points[i].y - cvGetReal1D( _pKalman[j]->state_pre, 1 ), 2 ) );
            if ( dist < minDist )
            {
                minDist = dist;
                minJ = j;
            }
        }

        if ( minJ >= 0 )
        {
            //
            // If exist, update the score and position
            //
            _candidateVelocity[minJ][0] = points[i].x - _candidatePoints[minJ].x;
            _candidateVelocity[minJ][1] = points[i].y - _candidatePoints[minJ].y;

            _candidatePoints[minJ].x = points[i].x;
            _candidatePoints[minJ].y = points[i].y;

            _candidateDist[minJ] = dist[i];

            _candidateScore[minJ] ++;
            _candidateLost [minJ] = 0;

            // Kalman correct
            float pt[2];
            pt[0] = _candidatePoints[minJ].x;
            pt[1] = _candidatePoints[minJ].y;
            CvMat measurementMat = cvMat( 2, 1, CV_32FC1, pt ); // measurement (x,y)
            cvKalmanCorrect( _pKalman[minJ], &measurementMat );

            _Rng.disttype = CV_RAND_NORMAL;
            cvRandSetRange( &_Rng, 0, sqrt(_pKalman[minJ]->process_noise_cov->data.fl[0]), 0 );
            cvRand( &_Rng, _pKalmanMat_W );
            cvMatMulAdd(
                _pKalman[minJ]->transition_matrix,
                _pKalman[minJ]->state_post,
                _pKalmanMat_W,
                _pKalman[minJ]->state_post );

        }
        else if ( _nPoints < NUM_MAX_CANDIDATES )
        {
            //
            // If there is no close point, add a new candidate
            //
            _candidatePoints[_nPoints].x = points[i].x;
            _candidatePoints[_nPoints].y = points[i].y;
            _candidateScore [_nPoints]   = 0;
            _candidateAge   [_nPoints]   = 0;
            _candidateLost  [_nPoints]   = 0;

            // Init Kalman filter
            _pKalman[_nPoints] = cvCreateKalman( 4, 2, 0 );
            cvCopy( _pKalmanMat_A, _pKalman[_nPoints]->transition_matrix );
            cvSetIdentity( _pKalman[_nPoints]->measurement_matrix, cvRealScalar(1) );
            cvSetIdentity( _pKalman[_nPoints]->process_noise_cov, cvRealScalar(1e-5) );
            cvSetIdentity( _pKalman[_nPoints]->measurement_noise_cov, cvRealScalar(1e-1) );
            cvSetIdentity( _pKalman[_nPoints]->error_cov_post, cvRealScalar(1) );
            cvSetReal1D( _pKalman[_nPoints]->state_post, 0, points[i].x );
            cvSetReal1D( _pKalman[_nPoints]->state_post, 1, points[i].y );
            cvSetReal1D( _pKalman[_nPoints]->state_post, 2, 0 );
            cvSetReal1D( _pKalman[_nPoints]->state_post, 3, 0 );

            _nPoints ++;
        }
    }/**/

    //
    // Drop candidate points by lost values and score
    //
    for ( int i = 0 ; i < _nPoints ; i ++ )
    {
        if ( _candidateLost[i] > THRESHOLD_LOST_TRACKING ||
             _candidateAge[i] > MIN_AGE_TO_TRACK &&
             (float)_candidateScore[i] / _candidateAge[i] < 0.5 )
        {
            _candidatePoints[i] = _candidatePoints[_nPoints-1];
            _candidateScore [i] = _candidateScore [_nPoints-1];
            _candidateAge   [i] = _candidateAge   [_nPoints-1];
            _candidateLost  [i] = _candidateLost  [_nPoints-1];
            _candidateDist  [i] = _candidateDist  [_nPoints-1];

            _nPoints--;
            i--;
        }
    }

    //
    // Increase Ages
    //
    for ( int i = 0 ; i < _nPoints ; i ++ )
    {
        _candidateAge[i] ++;
    }

    //
    // Sort candidates by their score
    //
/*    for ( int i = 0 ; i < _nPoints-1 ; i ++ )
    {
        for ( int j = i + 1 ; j < _nPoints ; j ++ )
        {
            if ( _candidateAge[j] > MIN_AGE_TO_TRACK &&
                 (float)_candidateScore[i]/_candidateAge[i] < (float)_candidateScore[j]/_candidateAge[j] )
            {
                CvPoint2D32f tempPoint = _candidatePoints[i];
                _candidatePoints[i] = _candidatePoints[j];
                _candidatePoints[j] = tempPoint;

                int tempScore = _candidateScore[i];
                _candidateScore[i] = _candidateScore[j];
                _candidateScore[j] = tempScore;

                int tempAge = _candidateAge[i];
                _candidateAge[i] = _candidateAge[j];
                _candidateAge[j] = tempAge;

                float tempDist = _candidateDist[i];
                _candidateDist[i] = _candidateDist[j];
                _candidateDist[j] = tempDist;
            }
        }
    }/**/

    //
    // Determine that 5 fingertips are detected
    //
/*    bool fDetected = true;
    for ( int i = 0 ; i < 5 ;  i ++ )
    {
        if ( _candidateAge[i] < THRESHOLD_AGE_TO_DETECT )
            fDetected = false;
    }
/**/

    //
    // Make Fingertip Template for the detected fingertips
    //
    _fDetected   = false;
    _nFingertips = 0;
    for ( int i = 0 ; i < _nPoints ; i ++ )
    {
        if ( _candidateAge[i] > THRESHOLD_AGE_TO_DETECT &&
             _candidatePoints[i].y > currCentroid.y )
        {
            //
            // Detected !
            //
            _nFingertipIndex[_nFingertips] = i;
            _fingertipPoints[_nFingertips] = _candidatePoints[i];
            _fingertipDist  [_nFingertips] = _candidateDist[i];
            
/*
            //
            // Make a template
            //
            MakeFingertipTemplate(
                image,
                cvPointFrom32f( _candidatePoints[i] ),
                _candidateDist[i],
                &(_pFingertipTemplate[ _nFingertips ]) );/**/

            _nFingertips ++;

            if ( _nFingertips == 5 )
            {
                _fDetected = true;
                break;
            }
        }
    }

    //
    // When detected, assign the fingertip index number
    //
    if ( _fDetected )
    {
        for ( int i = 0 ; i < _nFingertips-1 ; i ++ )
        {
            for ( int j = i + 1 ; j < _nFingertips ; j ++ )
            {
                // Sort by x-coordinate (just for simplicity now)
                if ( _fingertipPoints[i].x > _fingertipPoints[j].x )
                {
                    CvPoint2D32f tempPoint = _fingertipPoints[i];
                    _fingertipPoints[i] = _fingertipPoints[j];
                    _fingertipPoints[j] = tempPoint;

                    int tempIndex = _nFingertipIndex[i];
                    _nFingertipIndex[i] = _nFingertipIndex[j];
                    _nFingertipIndex[j] = tempIndex;

                    float tempDist = _fingertipDist[i];
                    _fingertipDist[i] = _fingertipDist[j];
                    _fingertipDist[j] = tempDist;

                    IplImage * tempImage = _pFingertipTemplate[i];
                    _pFingertipTemplate[i] = _pFingertipTemplate[j];
                    _pFingertipTemplate[j] = tempImage;
                }
            }
        }

        //
        // Determin the thumb based on the distance to the mean point
        //

        // compute the mean
        float mean_x = 0;
        float mean_y = 0;
        for ( int i = 0 ; i < _nFingertips ; i ++ )
        {
            mean_x += _fingertipPoints[i].x;
            mean_y += _fingertipPoints[i].y;
        }
        mean_x /= _nFingertips;
        mean_y /= _nFingertips;

        // get the farthest fingertip
        float maxDist = 0;
        int maxDistIndex = 0;
        for ( int i = 0 ; i < _nFingertips ; i ++ )
        {
            float dist = sqrt(
                pow(mean_x - _fingertipPoints[i].x, 2) +
                pow(mean_y - _fingertipPoints[i].y, 2) );
            if ( dist > maxDist )
            {
                maxDist = dist;
                maxDistIndex = i;
            }
        }
        
        // flip the order if the thumb is ordered to the end.
        if ( maxDistIndex == 0 )
        {
            for ( int i = 0 ; i < _nFingertips / 2 ; i ++ )
            {
                int j = _nFingertips - i - 1;

                CvPoint2D32f tempPoint = _fingertipPoints[i];
                _fingertipPoints[i] = _fingertipPoints[j];
                _fingertipPoints[j] = tempPoint;

                int tempIndex = _nFingertipIndex[i];
                _nFingertipIndex[i] = _nFingertipIndex[j];
                _nFingertipIndex[j] = tempIndex;

                float tempDist = _fingertipDist[i];
                _fingertipDist[i] = _fingertipDist[j];
                _fingertipDist[j] = tempDist;

                IplImage * tempImage = _pFingertipTemplate[i];
                _pFingertipTemplate[i] = _pFingertipTemplate[j];
                _pFingertipTemplate[j] = tempImage;
            }

            _fFlipOrder = true;
        }
        else
        {
            _fFlipOrder = false;
        }
    }

    //
    // Update Mode
    //
    if ( _fDetected )
    {
        if ( _nMode == MODE_FINGERTIP_NONE )
            _nMode = MODE_FINGERTIP_FIRST_DETECTED;
        if ( _nMode == MODE_FINGERTIP_LOST_TRACKING )
            _nMode = MODE_FINGERTIP_TRACKING;
    }

    return _fDetected;
}

bool FingertipTracker::TrackFingertips3(
    int nPoints,
    CvPoint * points,
    float * distValue,
    CvPoint prevCentroid,
    CvPoint currCentroid )
{
{
    //
    // Init
    //
    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
        _fTracked[i] = false;

    //
    // Check
    //
    if ( nPoints < NUM_FINGERTIP )
    {
        _fDetected = false;
        _nFingertips = 0;
        _nPoints = 0;
        goto Finished;
    }

    //
    // Match points and fingertips
    //
    int index[5] = { 0 };
    int minIndex[5] = { 0 };
    float dist[5] = { 0 };
    float minCost = 1000000;
    for ( int pt0 = 0 ; pt0 < nPoints ; pt0 ++ )                    // first fingertip
    {
        index[0] = pt0 % nPoints;
        dist[0] = sqrt(
            pow( (float)points[index[0]].x - currCentroid.x - _fingertipPoints[0].x + prevCentroid.x, 2 ) +
            pow( (float)points[index[0]].y - currCentroid.y - _fingertipPoints[0].y + prevCentroid.y, 2 ) );

        for ( int pt1 = 1 ; pt1 <= nPoints - 4 ; pt1 ++ )            // second fingertip
        {
            index[1] = _fFlipOrder ? (pt0-pt1) % nPoints : (pt0+pt1) % nPoints;
            dist[1] = sqrt(
                pow( (float)points[index[1]].x - currCentroid.x - _fingertipPoints[1].x + prevCentroid.x, 2 ) +
                pow( (float)points[index[1]].y - currCentroid.y - _fingertipPoints[1].y + prevCentroid.y, 2 ) );

            for ( int pt2 = 1 ; pt2 <= nPoints - pt1 - 3 ; pt2 ++ )        // third fingertip
            {
                index[2] = _fFlipOrder ? (pt0-pt1-pt2) % nPoints : (pt0+pt1+pt2) % nPoints;
                dist[2] = sqrt(
                    pow( (float)points[index[2]].x - currCentroid.x - _fingertipPoints[2].x + prevCentroid.x, 2 ) +
                    pow( (float)points[index[2]].y - currCentroid.y - _fingertipPoints[2].y + prevCentroid.y, 2 ) );

                for ( int pt3 = 1 ; pt3 <= nPoints - pt1 - pt2 - 2 ; pt3 ++ )    // fourth fingertip
                {
                    index[3] = _fFlipOrder ? (pt0-pt1-pt2-pt3) % nPoints : (pt0+pt1+pt2+pt3) % nPoints;
                    dist[3] = sqrt(
                        pow( (float)points[index[3]].x - currCentroid.x - _fingertipPoints[3].x + prevCentroid.x, 2 ) +
                        pow( (float)points[index[3]].y - currCentroid.y - _fingertipPoints[3].y + prevCentroid.y, 2 ) );

                    for ( int pt4 = 1 ; pt4 <= nPoints - pt1 - pt2 - pt3 - 1 ; pt4 ++ )// fifth fingertip
                    {
                        index[4] = _fFlipOrder ? (pt0-pt1-pt2-pt3-pt4) % nPoints : (pt0+pt1+pt2+pt3+pt4) % nPoints;
                        dist[4] = sqrt(
                            pow( (float)points[index[4]].x - currCentroid.x - _fingertipPoints[4].x + prevCentroid.x, 2 ) +
                            pow( (float)points[index[4]].y - currCentroid.y - _fingertipPoints[4].y + prevCentroid.y, 2 ) );

                        //
                        // Compute the matching cost
                        //
                        float matchingCost = 0;
                        for ( int i = 0 ; i < 5 ; i ++ )
                        {
                            matchingCost += dist[i];
                        }

                        //
                        // Find minimum cost matching
                        //
                        if ( matchingCost < minCost )
                        {
                            minCost = matchingCost;
                            for ( int i = 0 ; i < 5 ; i ++ )
                            {
                                minIndex[i] = index[i];
                            }
                        }
                    }
                }
            }
        }
    }

    //
    // Update Tracking
    //
    for ( int i = 0 ; i < 5 ; i ++ )
    {
        _fingertipPoints[i].x = points[minIndex[i]].x;
        _fingertipPoints[i].y = points[minIndex[i]].y;
        _fingertipDist  [i]   = distValue[minIndex[i]];

        _fTracked[i] = true;
    }
}
Finished:

    //
    // Update Mode
    //
    if ( _fDetected )
    {
        _nMode = MODE_FINGERTIP_TRACKING;
    }
    else
    {
        _nMode = MODE_FINGERTIP_LOST_TRACKING;
    }

    return _fDetected;
}

bool FingertipTracker::TrackFingertips2(
    int nPoints,
    CvPoint * points,
    float * distValue,
    CvPoint prevCentroid,
    CvPoint currCentroid )
{
    //
    // Init
    //
    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
        _fTracked[i] = false;

    //
    // Match Correspondences to the fingertip points
    //
    bool fMatched[NUM_FINGERTIP] = { false };
    int  nTracked = 0;
    float mean_dx = 0;
    float mean_dy = 0;
    int prevJ = -1;
    for ( int i = 0 ; i < nPoints ; i ++ )
    {
        //
        // Find the closest point of each fingertip point relative to the centroids
        //
        int   minJ = -1;
        float minDist = THRESHOLD_CLOSEST_DIST;

        for ( int j = prevJ + 1 ; j < _nFingertips; j ++ )
        {
            float dist = sqrt(
                pow( (float)points[i].x - currCentroid.x - _fingertipPoints[j].x + prevCentroid.x, 2 ) +
                pow( (float)points[i].y - currCentroid.y - _fingertipPoints[j].y + prevCentroid.y, 2 ) );
            if ( dist < minDist )
            {
                minDist = dist;
                minJ = j;
            }
        }

        if ( minJ >= 0 )
        {
            //
            // If exist update the score and position
            //
            fMatched[minJ] = true;

            mean_dx += (_fingertipPoints[minJ].x - points[i].x);
            mean_dy += (_fingertipPoints[minJ].y - points[i].y);

            _fingertipPoints[minJ].x = points[i].x;
            _fingertipPoints[minJ].y = points[i].y;
            _fingertipDist  [minJ]   = distValue[minJ];

            _fTracked[minJ] = true;
            nTracked ++;

            prevJ = minJ;
        }
    }

    if ( nTracked <= 3 )
    {
        _fDetected = false;
        _nFingertips = 0;
        _nPoints = 0;
    }

    if ( nTracked == 4 )
    {
        for ( int i = 0 ; i < _nFingertips ; i ++ )
        {
            if ( _fTracked[i] == false )
            {
                _fingertipPoints[i].x -= (mean_dx/nTracked);
                _fingertipPoints[i].y -= (mean_dy/nTracked);
            }
        }
    }

    //
    // Update Mode
    //
    if ( _fDetected )
    {
        _nMode = MODE_FINGERTIP_TRACKING;
    }
    else
    {
        _nMode = MODE_FINGERTIP_LOST_TRACKING;
    }

    return _fDetected;
}

bool FingertipTracker::TrackFingertips( IplImage * image, int nPoints, CvPoint * points, float * distValue )
{
    //
    // Template Matching
    //
    MatchFingertipTemplate( image );

    //
    // Match Correspondences to the fingertip points
    //
    bool * fMatched = (bool *)malloc( nPoints * sizeof(bool) );
    memset( fMatched, 0, nPoints * sizeof(bool) );

    for ( int i = 0 ; i < _nFingertips ; i ++ )
    {
        //
        // Find the closest point of each fingertip point
        //
        int   minJ = -1;
        float minDist = THRESHOLD_CLOSEST_DIST2;
        if ( _pFingertipTemplate[i] == 0 ) minDist = THRESHOLD_CLOSEST_DIST;

        for ( int j = 0 ; j < nPoints ; j ++ )
        {
            float dist = sqrt(
                pow( (float)points[j].x - _fingertipPoints[i].x, 2 ) +
                pow( (float)points[j].y - _fingertipPoints[i].y, 2 ) );
            if ( !fMatched[j] && dist < minDist )
            {
                minDist = dist;
                minJ = j;
            }
        }

        if ( minJ >= 0 )
        {
            //
            // If exist update the score and position
            //
            fMatched[minJ] = true;

            _fingertipPoints[i].x = points[minJ].x;
            _fingertipPoints[i].y = points[minJ].y;
            _fingertipDist  [i]   = distValue[minJ];
        }
        else if ( _pFingertipTemplate[i] == 0 )
        {
            //
            // Now, we don't have a valid template or a matched candidate.
            // We can say that we need to reinitialize the hand detection.
            //
            _fDetected = false;
            _nFingertips = 0;
            _nPoints = 0;
        }
    }

    //
    // Release Memory
    //
    free( fMatched );

    //
    // Make a template
    //
    for ( int i = 0 ; i < _nFingertips ; i ++ )
    {
        MakeFingertipTemplate(
            image,
            cvPointFrom32f( _fingertipPoints[i] ),
            _fingertipDist[i],
            &(_pFingertipTemplate[ i ]) );
    }


    return _fDetected;
}

void FingertipTracker::MakeFingertipTemplate( IplImage * image, CvPoint point, float dist, IplImage ** pTemplate )
{
    //
    // Set Template Rect
    //
    CvSize size = cvSize( cvRound(dist*2+5), cvRound(dist*2+5) );
    if ( size.width > MAX_TEMPLATE_SIZE ) size = cvSize( MAX_TEMPLATE_SIZE, MAX_TEMPLATE_SIZE );

    CvRect templateRect;
    templateRect.x = point.x - size.width / 2;
    templateRect.y = point.y - size.height/ 2;
    templateRect.width = size.width;
    templateRect.height = size.height;
    if ( templateRect.x < 0 || templateRect.y < 0 ||
         templateRect.x + size.width >= image->width ||
         templateRect.y + size.height>= image->height )
    {
        // Out of boundary, nullify the template
        cvReleaseImage( pTemplate );
        *pTemplate = 0;

        // Return !!! don't make a template
        return;
    }

    //
    // (Re)allocate memory if necessary
    //
    if ( (*pTemplate) != 0 &&
         ( (*pTemplate)->width != templateRect.width ||
           (*pTemplate)->height!= templateRect.height )
       )
    {
        // Already exist, and need to change the size
        cvReleaseImage( pTemplate );
        *pTemplate = 0;
    }
    if ( (*pTemplate) == 0 )
    {
        // New allocation
        *pTemplate = cvCreateImage( cvSize(templateRect.width, templateRect.height), 8, image->nChannels );
    }

    //
    // Copy the image patch
    //
    cvSetImageROI( image, templateRect );
    cvCopyImage( image, *pTemplate );
    cvResetImageROI( image );
}

CvPoint2D32f * FingertipTracker::QueryFingertip( int index )
{
    if ( index >= _nFingertips ) return 0;

    return &(_fingertipPoints[ index ]);
}

float FingertipTracker::QueryFingertipDist( int index )
{
    if ( index >= _nFingertips ) return 0;

    return _fingertipDist[index];
}

CvPoint2D32f FingertipTracker::QueryFingertipKalman(int index)
{
    CvPoint2D32f pt = cvPoint2D32f( 0, 0 );
    if ( index >= _nPoints ) return pt;

//    pt.x = cvGetReal1D( _pKalman[index]->state_post, 0 );
//    pt.y = cvGetReal1D( _pKalman[index]->state_post, 1 );

    return pt;
}

void FingertipTracker::MatchFingertipTemplate( IplImage * image )
{
    //
    // Match Fingertip Templates
    //
    for ( int i = 0 ; i < _nFingertips ; i ++ )
    {
        //
        // Check if the fingertip has a valid template
        //
        if ( !_pFingertipTemplate[i] ) continue;

        //
        // Set Search rect considering the boundary
        //
        CvRect searchRect;
        searchRect.x = _fingertipPoints[i].x - TEMPLATE_SEARCH_WINDOW/2 - _pFingertipTemplate[i]->width / 2;
        searchRect.y = _fingertipPoints[i].y - TEMPLATE_SEARCH_WINDOW/2 - _pFingertipTemplate[i]->height/ 2;
        searchRect.width  = TEMPLATE_SEARCH_WINDOW + _pFingertipTemplate[i]->width;
        searchRect.height = TEMPLATE_SEARCH_WINDOW + _pFingertipTemplate[i]->height;
        if ( searchRect.x < 0 ) searchRect.x = 0;
        if ( searchRect.y < 0 ) searchRect.y = 0;
        if ( searchRect.x + searchRect.width >= image->width ) searchRect.x = image->width - searchRect.width - 1;
        if ( searchRect.y + searchRect.height>= image->height) searchRect.y = image->height- searchRect.height- 1;

        //
        // Prepare result matrix
        //
        CvMat * pResultMat = cvCreateMat( TEMPLATE_SEARCH_WINDOW + 1, TEMPLATE_SEARCH_WINDOW + 1, CV_32FC1 );

        //
        // Matching
        //
        cvSetImageROI( image, searchRect );
        cvMatchTemplate( image, _pFingertipTemplate[i], pResultMat, CV_TM_CCORR_NORMED );
        cvResetImageROI( image );

        //
        // Get Max Point
        //
        double maxValue;
        CvPoint maxPoint;
        cvMinMaxLoc( pResultMat, 0, &maxValue, 0, &maxPoint );

        //
        // Release Memory
        //
        cvReleaseMat( &pResultMat );

        //
        // Update to the matched point
        //
        float newX = _fingertipPoints[i].x + maxPoint.x - TEMPLATE_SEARCH_WINDOW/2;
        float newY = _fingertipPoints[i].y + maxPoint.y - TEMPLATE_SEARCH_WINDOW/2;
        if ( maxValue > THRESHOLD_TEMPLATE_MATCH &&
             newX > _pFingertipTemplate[i]->width &&
             newX + _pFingertipTemplate[i]->width < image->width &&
             newY > _pFingertipTemplate[i]->height &&
             newY + _pFingertipTemplate[i]->height < image->height
           )
        {
            _fingertipPoints[i].x = newX;
            _fingertipPoints[i].y = newY;

            _candidatePoints[_nFingertipIndex[i]].x = _fingertipPoints[i].x;
            _candidatePoints[_nFingertipIndex[i]].y = _fingertipPoints[i].y;
        }
        else
        {
            // If the match score is bad or the position is at boundary, nullify the template.
            cvReleaseImage( &_pFingertipTemplate[i] );
            _pFingertipTemplate[i] = 0;
        }
    }
}

void FingertipTracker::PredictFingertips(
    const CvMat * intrinsic_matrix,
    const CvMat * distortion_coeffs,
    const CvMat * rotation_vector,
    const CvMat * translation_vector )
{
    //
    // Check availability
    //
    if ( _fFingertipCoordinates == false )
        return;

    //
    // Prepare Fingertip Coordinates ( object points )
    //
    CvMat objectMat = cvMat( NUM_FINGERTIP, 3, CV_32FC1, _fingertipCoordinates );

    //
    // Prepare Fingertip points ( image points )
    //
    CvMat imageMat = cvMat( NUM_FINGERTIP, 2, CV_32FC1, _fingertipPoints );

    //
    // Project the fingertips to the image
    //
    cvProjectPoints2(
        &objectMat,
        rotation_vector,
        translation_vector,
        intrinsic_matrix,
        distortion_coeffs,
        &imageMat );

}

bool FingertipTracker::FindExtrinsicCameraParams(
    const CvMat *intrinsic_matrix,
    const CvMat *distortion_coeffs,
          CvMat *rotation_vector,
          CvMat *translation_vector )
{
    //
    // Check availability
    //
    if ( _nFingertips != NUM_FINGERTIP ||
         _fFingertipCoordinates == false )
         return false;

    //
    // Prepare Fingertip Coordinates ( object points )
    //
    CvMat objectMat = cvMat( NUM_FINGERTIP, 3, CV_32FC1, _fingertipCoordinates );

    //
    // Prepare Fingertip points ( image points )
    //
#ifdef PROCESS_320x240
    float fingertipPoints[NUM_FINGERTIP][2];
    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
    {
        fingertipPoints[i][0] = _fingertipPoints[i].x * 2;
        fingertipPoints[i][1] = _fingertipPoints[i].y * 2;
    }
    CvMat imageMat = cvMat( NUM_FINGERTIP, 2, CV_32FC1, fingertipPoints );
#else
    CvMat imageMat = cvMat( NUM_FINGERTIP, 2, CV_32FC1, _fingertipPoints );
#endif

/**/
    /*
    //
    // Prepare Fingertip Coordinates ( object points ) with extra points
    //
    float fingertipCoordinates[NUM_FINGERTIP*5][3];
    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
    {
        for ( int j = 0 ; j < 3 ; j ++ )
        {
            fingertipCoordinates[i*5+0][j] = _fingertipCoordinates[i][j];
            fingertipCoordinates[i*5+1][j] = _fingertipCoordinates[i][j];
            fingertipCoordinates[i*5+2][j] = _fingertipCoordinates[i][j];
            fingertipCoordinates[i*5+3][j] = _fingertipCoordinates[i][j];
            fingertipCoordinates[i*5+4][j] = _fingertipCoordinates[i][j];
        }
    }
    CvMat objectMat = cvMat( NUM_FINGERTIP * 5, 3, CV_32FC1, fingertipCoordinates );

    //
    // Prepare Fingertip points ( image points )
    //
    float fingertipPoints[NUM_FINGERTIP*5][2];
    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
    {
        fingertipPoints[i*5+0][0] = _fingertipPoints[i].x;
        fingertipPoints[i*5+1][0] = _fingertipPoints[i].x-1;
        fingertipPoints[i*5+2][0] = _fingertipPoints[i].x;
        fingertipPoints[i*5+3][0] = _fingertipPoints[i].x+1;
        fingertipPoints[i*5+4][0] = _fingertipPoints[i].x;

        fingertipPoints[i*5+0][1] = _fingertipPoints[i].y;
        fingertipPoints[i*5+1][1] = _fingertipPoints[i].y;
        fingertipPoints[i*5+2][1] = _fingertipPoints[i].y+1;
        fingertipPoints[i*5+3][1] = _fingertipPoints[i].y;
        fingertipPoints[i*5+4][1] = _fingertipPoints[i].y-1;
    }
    CvMat imageMat = cvMat( NUM_FINGERTIP * 5, 2, CV_32FC1, fingertipPoints );
    /**/

    //
    // Call OpenCV function
    //
    /*
    bool fPoseEstimated =
        cvFindExtrinsicCameraParams3(
            &objectMat,
            &imageMat,
            intrinsic_matrix,
            distortion_coeffs,
            rotation_vector,
            translation_vector );
     */
     bool fPoseEstimated = true;
     cvFindExtrinsicCameraParams2(
         &objectMat,
         &imageMat,
         intrinsic_matrix,
         distortion_coeffs,
         rotation_vector,
         translation_vector );

/*    //
    // for verbose, print results
    //
    printf( "r( " );
    for ( int i = 0 ; i < 3 ; i ++ )
        printf( "%.3f ", rotation_vector->data.fl[i] );
    printf( ")  t( ");
    for ( int i = 0 ; i < 3 ; i ++ )
        printf( "%.3f ", translation_vector->data.fl[i] );
    printf( ")\n" ); /**/

    return fPoseEstimated;
}

void FingertipTracker::ResetCoordinates()
{
    _fFingertipCoordinates = false;
    _fInitialFingertip = false;
    _numCoordinateSamples = 0;
    if ( _fpCoordinates )
    {
        fclose( _fpCoordinates );
    }
    _fpCoordinates = fopen( "coordinates.txt", "wt" );
}

bool FingertipTracker::CalculateCoordinates( const CvMat * ProjectionMat )
{
    //
    // Unproject the image points with a certain height constraint
    //
    float X, Y, Z;
    float p[3][4];
    for ( int i = 0 ; i < 3 ; i ++ )
        for ( int j = 0 ; j < 4 ; j ++ )
            p[i][j] = cvGetReal2D( ProjectionMat, i, j );

    for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
    {
#ifdef PROCESS_320x240
        float x = _fingertipPoints[i].x * 2;
        float y = _fingertipPoints[i].y * 2;
#else
        float x = _fingertipPoints[i].x;
        float y = _fingertipPoints[i].y;
#endif

        float p11_p31x = p[0][0] - p[2][0] * x;
        float p22_p32y = p[1][1] - p[2][1] * y;
        float p21_p31y = p[1][0] - p[2][0] * y;
        float p12_p32x = p[0][1] - p[2][1] * x;
        float p13_p33x = p[0][2] - p[2][2] * x;
        float p23_p33y = p[1][2] - p[2][2] * y;
        float p14_p34x = p[0][3] - p[2][3] * x;
        float p24_p34y = p[1][3] - p[2][3] * y;

        Z = 5.0;
        X = ( - p13_p33x * p22_p32y * Z + p23_p33y * p12_p32x * Z
              - p14_p34x * p22_p32y     + p24_p34y * p12_p32x ) /
            (   p11_p31x * p22_p32y     - p21_p31y * p12_p32x );

        Y = ( - p11_p31x * X - p13_p33x * Z - p14_p34x ) / p12_p32x;

        if ( _fInitialFingertip == false )
        {
            _fingertipCoordinates[i][0] = X;
            _fingertipCoordinates[i][1] = Y;
            _fingertipCoordinates[i][2] = Z;
            _numCoordinateSamples = 1;
        }
        else
        {
            _fingertipCoordinates[i][0] = ( _numCoordinateSamples * _fingertipCoordinates[i][0] + X ) / ( _numCoordinateSamples + 1 );
            _fingertipCoordinates[i][1] = ( _numCoordinateSamples * _fingertipCoordinates[i][1] + Y ) / ( _numCoordinateSamples + 1 );
            _fingertipCoordinates[i][2] = ( _numCoordinateSamples * _fingertipCoordinates[i][2] + Z ) / ( _numCoordinateSamples + 1 );

            //if ( _numCoordinateSamples < 50 )
                _numCoordinateSamples += 1;
        }

        printf("Unproject(%d) = ( %.3f, %.3f, %.3f )\n", i, _fingertipCoordinates[i][0], _fingertipCoordinates[i][1], _fingertipCoordinates[i][2] );
        fprintf( _fpCoordinates, "%.3f\t%.3f\t%.3f\n", _fingertipCoordinates[i][0], _fingertipCoordinates[i][1], _fingertipCoordinates[i][2] );
    }
    
    _fInitialFingertip = true;
    _fFingertipCoordinates = true;

    //
    // Calculate the center of the fingertips
    //

    // center x = mid-finger's x position
    _centerCoordinates[0] = _fingertipCoordinates[NUM_FINGERTIP/2][0];
    // center y = last-finger(thumb)'s y position
    _centerCoordinates[1] = _fingertipCoordinates[NUM_FINGERTIP-1][1];
    // center z = 0
    _centerCoordinates[2] = _fingertipCoordinates[0][2];

    return true;
}

void FingertipTracker::CentralizeCoordinates()
{
    //
    // Translate to the center
    //
    for ( int i = 0 ; i < 3 ; i ++ )
    {
        for ( int j = 0 ; j < NUM_FINGERTIP ; j ++ )
        {
            _fingertipCoordinates[j][i] -= _centerCoordinates[i];
        }
    }
}

float * FingertipTracker::QueryCenterCoordinates()
{
    return _centerCoordinates;
}

bool FingertipTracker::TriangulateCoordinates( const CvMat *ProjectionMat )
{
    bool fTriangulateCompleted = false;

    if ( _fInitialFingertip == false )
    {
        //
        // First, we keep the fingertip image points
        //
        for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
        {
            _InitialFingertipPoints[i] = _fingertipPoints[i];
        }
        for ( int i = 0 ; i < 3 ; i ++ )
            for ( int j = 0 ; j < 4 ; j ++ )
                _InitialProjectionMatrix[i][j] = cvGetReal2D( ProjectionMat, i, j );

        _fInitialFingertip = true;
    }
    else
    {
        //
        // When we have the initial fingertip, we triangulate each point
        //
        float newCoordinates[NUM_FINGERTIP][3];

        for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
        {
            // Stack a Linear system
            float a[4][4];
            CvMat A = cvMat( 4, 4, CV_32FC1, a );
            for ( int j = 0 ; j < 4 ; j ++ )
            {
                a[0][j] = _InitialFingertipPoints[i].x * _InitialProjectionMatrix[2][j] - _InitialProjectionMatrix[0][j];
                a[1][j] = _InitialFingertipPoints[i].y * _InitialProjectionMatrix[2][j] - _InitialProjectionMatrix[1][j];
                a[2][j] = _fingertipPoints[i].x * cvGetReal2D( ProjectionMat, 2, j ) - cvGetReal2D( ProjectionMat, 0, j );
                a[3][j] = _fingertipPoints[i].y * cvGetReal2D( ProjectionMat, 2, j ) - cvGetReal2D( ProjectionMat, 1, j );
            }

            // SVD: A = U W Trans(V)
            float w[4];
            CvMat W = cvMat( 4, 1, CV_32FC1, w );
            float v_t[4][4];
            CvMat V_T = cvMat( 4, 4, CV_32FC1, v_t );
            cvSVD( &A, &W, NULL, &V_T, CV_SVD_V_T );

            // Take new Coordinates
            for ( int j = 0 ; j < 3 ; j ++ )
            {
                newCoordinates[i][j] = v_t[3][j] / v_t[3][3];
            }
        }

        //
        // Selectively update the coordinates
        //
        int nValidCoordinates = 0;
        for ( int i = 0 ; i < NUM_FINGERTIP ; i ++ )
        {
            if ( 1 )/* newCoordinates[i][2] >= -50.0 &&
                 newCoordinates[i][2] <=  50.0 )*/
            {
                nValidCoordinates ++;

                // for simplicity now
                // BUGBUG: Kalman filter maybe
                if ( _fFingertipCoordinates )
                {
                    _fingertipCoordinates[i][0] = newCoordinates[i][0];
                    _fingertipCoordinates[i][1] = newCoordinates[i][1];
                    _fingertipCoordinates[i][2] = newCoordinates[i][2];
                }
                else
                {
                    _fingertipCoordinates[i][0] = ( _fingertipCoordinates[i][0] + newCoordinates[i][0] ) / 2.0;
                    _fingertipCoordinates[i][1] = ( _fingertipCoordinates[i][1] + newCoordinates[i][1] ) / 2.0;
                    _fingertipCoordinates[i][2] = ( _fingertipCoordinates[i][2] + newCoordinates[i][2] ) / 2.0;
                }
                _fingertipCoordinates[i][2] = -5;
                printf("Triangulate(%d) = ( %.3f, %.3f, %.3f )\n", i, _fingertipCoordinates[i][0], _fingertipCoordinates[i][1], _fingertipCoordinates[i][2] );
            }
            else if ( _fFingertipCoordinates )
            {
                nValidCoordinates ++;
            }
            else
            {
                printf("Ignore(%d) = ( %.3f, %.3f, %.3f )\n", i, newCoordinates[i][0], newCoordinates[i][1], newCoordinates[i][2] );
            }
        }
        if ( nValidCoordinates == NUM_FINGERTIP )
        {
            _fFingertipCoordinates = true;
        }
    }

    return fTriangulateCompleted;
}
