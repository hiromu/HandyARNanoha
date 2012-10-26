#include <HandyAR/HandRegion.h>
#include <stdio.h>

HandRegion::HandRegion(void)
{
    _pImage = 0;
    _pImageGradient = 0;
    _pImageHandMask = 0;
    _pImageYCrCb = 0;

#ifndef USE_GAUSSIAN_MODEL

    _pHSV = 0;
    _pH = 0, _pS = 0, _pV = 0;
    _pHmin = 0, _pHmax = 0, _pSmin = 0, _pSmax = 0, _pVmin = 0, _pVmax = 0;

#endif

    // Histogram
    for ( int i = 0 ; i < 16 ; i ++ )
        for ( int j = 0 ; j < 16 ; j ++ )
            for ( int k = 0 ; k < 16 ; k ++ )
                _histPixel[i][j][k] = 0;
    _nTotalPixel = 0;

    _fLearned = false;

}

HandRegion::~HandRegion(void)
{
    if ( _pImage )
    {
        cvReleaseImage( &_pImage );
    }
    if ( _pImageGradient )
    {
        cvReleaseImage( &_pImageGradient );
    }
    if ( _pImageHandMask )
    {
        cvReleaseImage( &_pImageHandMask );
    }
    if ( _pImageYCrCb )
    {
        cvReleaseImage( &_pImageYCrCb );
    }

#ifndef USE_GAUSSIAN_MODEL

    if ( _pHSV )
    {
        cvReleaseImage( &_pHSV );
    }
    if ( _pH )
    {
        cvReleaseImage( &_pH );
    }
    if ( _pS )
    {
        cvReleaseImage( &_pS );
    }
    if ( _pV )
    {
        cvReleaseImage( &_pV );
    }
    if ( _pHmin )
    {
        cvReleaseImage( &_pHmin );
    }
    if ( _pHmax )
    {
        cvReleaseImage( &_pHmax );
    }
    if ( _pSmin )
    {
        cvReleaseImage( &_pSmin );
    }
    if ( _pSmax )
    {
        cvReleaseImage( &_pSmax );
    }
    if ( _pVmin )
    {
        cvReleaseImage( &_pVmin );
    }
    if ( _pVmax )
    {
        cvReleaseImage( &_pVmax );
    }
    if ( _pHImage )
    {
        cvReleaseImage( &_pHImage );
    }
    if ( _pSImage )
    {
        cvReleaseImage( &_pSImage );
    }
    if ( _pVImage )
    {
        cvReleaseImage( &_pVImage );
    }

#endif

}

bool HandRegion::LoadColorConfig()
{
    FILE *fp = fopen("color.txt", "r");
    if(!fp)
        return false;
    fscanf(fp, "%d%d%d%d%d%d", &hmin, &smin, &vmin, &hmax, &smax, &vmax);
    return true;
}

bool HandRegion::LoadSkinColorProbTable()
{
    if ( _SkinColor.LoadLookUpTable( "skin.dis" ) == false )
    {
        if ( _SkinColor.LoadFile( "skin.mgm" ) == false )
        {
            fprintf( stderr, "skin color distribution load error.\n" );
            return false;
        }
        printf("making a lookup table for skin color distribution ");
        _SkinColor.MakeLookUpTable();
        printf("done\n");
        if ( _SkinColor.SaveLookUpTable( "skin.dis" ) == false )
        {
            fprintf( stderr, "skin color distribution look up table save error.\n" );
            return false;
        }
    }
    if ( _NonSkinColor.LoadLookUpTable( "nonskin.dis" ) == false )
    {
        if ( _NonSkinColor.LoadFile( "nonskin.mgm" ) == false )
        {
            fprintf( stderr, "non-skin color distribution load error.\n" );
            return false;
        }
        printf("making a lookup table for non-skin color distribution ");
        _NonSkinColor.MakeLookUpTable();
        printf("done\n");
        if ( _NonSkinColor.SaveLookUpTable( "nonskin.dis" ) == false )
        {
            fprintf( stderr, "non-skin color distribution look up table save error.\n" );
            return false;
        }
    }

    return true;
}

bool HandRegion::LoadHandMask( char * filename )
{
    //
    // Initialize Memory for Image
    //
    if ( !_pImageHandMask )
    {
        _pImageHandMask = cvLoadImage( filename, CV_LOAD_IMAGE_GRAYSCALE );
        if ( _pImageHandMask == 0 )
            return false;
        cvFlip( _pImageHandMask );
    }

    return true;
}

bool HandRegion::InitHandMask( IplImage * srcImage )
{
    //
    // Initialize Memory for Hand Mask Image
    //
    if ( !_pImageHandMask )
    {
        _pImageHandMask = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
        if ( _pImageHandMask == 0 )
            return false;
        _pImageHandMask->origin = srcImage->origin;
        cvSetZero( _pImageHandMask );
    }

    return true;
}

IplImage * HandRegion::GetHandRegion( IplImage * srcImage, IplImage * srcGrayImage, bool fScreenshot )
{
    //
    // Initialize Memory for Image
    //
    if ( !_pImage )
    {
        _pImage = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
        _pImage->origin = srcImage->origin;
    }
    if ( !_pImageGradient )
    {
        _pImageGradient = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
        _pImageGradient->origin = srcImage->origin;
    }
    if ( !_pImageYCrCb )
    {
        _pImageYCrCb = cvCreateImage( cvGetSize( srcImage ), 8, 3 );
        _pImageYCrCb->origin = srcImage->origin;
    }
    IplImage * pScreenshot = 0;
    if ( fScreenshot )
    {
        pScreenshot = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
        pScreenshot->origin = srcImage->origin;
    }

    //
    // Convert BGR -> YCrCb
    //
    cvCvtColor( srcImage, _pImageYCrCb, CV_BGR2YCrCb );

    //
    // Gradient
    //
    //cvCanny( srcGrayImage, _pImageGradient, 100, 200 );

#ifndef USE_GAUSSIAN_MODEL

    if ( !_pHSV )
    {
        _pHSV = cvCreateImage( cvGetSize( srcImage ), 8, 3 );
    }
    if ( !_pH )
    {
        _pH = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pS )
    {
        _pS = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pV )
    {
        _pV = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pHmin )
    {
        _pHmin = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pHmax )
    {
        _pHmax = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pSmin )
    {
        _pSmin = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pSmax )
    {
        _pSmax = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pVmin )
    {
        _pVmin = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pVmax )
    {
        _pVmax = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pHImage )
    {
        _pHImage = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pSImage )
    {
        _pSImage = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }
    if ( !_pVImage )
    {
        _pVImage = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
    }

    cvCvtColor( srcImage, _pHSV, CV_BGR2RGB );
    cvSplit( _pHSV, _pH, _pS, _pV, NULL );
    cvThreshold( _pH, _pHmin, hmin, 255, CV_THRESH_BINARY );
    cvThreshold( _pH, _pHmax, hmax, 255, CV_THRESH_BINARY_INV );
    cvThreshold( _pS, _pSmin, smin, 255, CV_THRESH_BINARY );
    cvThreshold( _pS, _pSmax, smax, 255, CV_THRESH_BINARY_INV );
    cvThreshold( _pV, _pVmin, vmin, 255, CV_THRESH_BINARY );
    cvThreshold( _pV, _pVmax, vmax, 255, CV_THRESH_BINARY_INV );
    cvAnd( _pHmin, _pHmax, _pHImage );
    cvAnd( _pSmin, _pSmax, _pSImage );
    cvAnd( _pVmin, _pVmax, _pVImage );
    cvAnd( _pHImage, _pSImage, _pImage );
    cvAnd( _pImage, _pVImage, _pImage );

    cvNot( _pImage, _pImage );

    cvSaveImage( "debug.png", _pImage );
    cvSaveImage( "Hdebug.png", _pHImage );
    cvSaveImage( "Sdebug.png", _pSImage );
    cvSaveImage( "Vdebug.png", _pVImage );

#else

    //
    // Segmentation by Color Distribution
    //
    int nHandRegion = 0;
    for ( int i = 0 ; i < srcImage->height ; i ++ )
    {
        for ( int j = 0 ; j < srcImage->width ; j ++ )
        {
            unsigned char R = srcImage->imageData[ i * srcImage->widthStep + j*3 + 2];
            unsigned char G = srcImage->imageData[ i * srcImage->widthStep + j*3 + 1];
            unsigned char B = srcImage->imageData[ i * srcImage->widthStep + j*3 + 0];

            float P_Skin = _SkinColor.GetProbabilityByLookup( R, G, B );
            float P_NonSkin = _NonSkinColor.GetProbabilityByLookup( R, G, B );

            if ( _fLearned )
            {
//                P_Skin = (P_Skin + 4*(float)_histPixel[R/16][G/16][B/16] / _nTotalPixel )/5;

                unsigned char Y  = _pImageYCrCb->imageData[ i * _pImageYCrCb->widthStep + j * 3 + 0 ];
                unsigned char Cr = _pImageYCrCb->imageData[ i * _pImageYCrCb->widthStep + j * 3 + 1 ];
                unsigned char Cb = _pImageYCrCb->imageData[ i * _pImageYCrCb->widthStep + j * 3 + 2 ];

                float P_Skin_hist = _ColorHistogram.QueryProbability( Y, Cr, Cb );
                P_Skin = (1 - ALPHA) * P_Skin + ALPHA * P_Skin_hist;

//                P_Skin = (P_Skin + 9*(float)_histPixelCrCb[Y/16][Cr/16][Cb/16] / _nTotalPixel )/10;
            }
//            cvSetReal2D( _pImage, i, j, P_Skin*255 );

            float P = P_Skin/P_NonSkin;
            if ( P < 0.4 ) {//|| cvGetReal2D( _pImageGradient, i, j ) > 0 ) {
                _pImage->imageData[ i * _pImage->widthStep + j ] = 0;
            }
            else
            {
                _pImage->imageData[ i * _pImage->widthStep + j ] = 255;
                nHandRegion ++;
            }/**/
            if ( fScreenshot )
            {
                P = sqrt(sqrt(P)) * 255;
                if ( P > 255 ) P = 255;
                pScreenshot->imageData[ i * pScreenshot->widthStep + j ] = P;
            }
        }
    }

#endif

    //
    // Smoothing and Thresholding
    //
    cvSmooth( _pImage, _pImage, CV_BLUR, 5, 5 );
    cvThreshold( _pImage, _pImage, 128, 255, CV_THRESH_BINARY );

    //
    // Morphology Operation
    //
    cvMorphologyEx( _pImage, _pImage, 0, 0, CV_MOP_OPEN, 1 );
    cvMorphologyEx( _pImage, _pImage, 0, 0, CV_MOP_CLOSE, 1 );

    //
    // Connected Component Filtering
    //
    /*
    _blobs = CBlobResult( _pImage, NULL, 100, true );
    CBlob largestBlob;
    _blobs.GetNthBlob( CBlobGetArea(), 0, largestBlob );
    cvSetZero( _pImage );
    largestBlob.FillBlob( _pImage, cvScalar(255) );
    _blobs.Filter( _blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 500 );
    for ( int i = 0 ; i < _blobs.GetNumBlobs() ; i ++ )
    {
        CBlob * pBlob = _blobs.GetBlob(i);
        if ( pBlob->Area() < 5000 )
        {
           pBlob->FillBlob( _pImage, cvScalar(0) );
        }
    }
    */

    if ( fScreenshot )
    {
        cvSaveImage( "screenshot_handprob.png", pScreenshot );
        cvReleaseImage( &pScreenshot );
    }

    return _pImage;
}

void HandRegion::LearnColorCrCb( bool fLearn )
{
    if ( !fLearn || !_pImageYCrCb || !_pImageHandMask )
    {
        _fLearned = false;
        _ColorHistogram.Reset();
        return;
    }
/*
    // Learn Histogram
    for ( int i = 0 ; i < 16 ; i ++ )
        for ( int j = 0 ; j < 16 ; j ++ )
            for ( int k = 0 ; k < 16 ; k ++ )
            _histPixelCrCb[i][j][k] = 0;
    _nTotalPixel = 0;
    for ( int i = 0 ; i < _pImageHandMask->width ; i ++ )
    {
        for ( int j = 0 ; j < _pImageHandMask->height ; j ++ )
        {
            if ( cvGetReal2D( _pImageHandMask, j, i ) == 255 )
            {
                unsigned char Y  = _pImageYCrCb->imageData[ j * _pImageYCrCb->widthStep + i * 3 + 0 ];
                unsigned char Cr = _pImageYCrCb->imageData[ j * _pImageYCrCb->widthStep + i * 3 + 1 ];
                unsigned char Cb = _pImageYCrCb->imageData[ j * _pImageYCrCb->widthStep + i * 3 + 2 ];

                _histPixelCrCb[Y/16][Cr/16][Cb/16] ++;
                _nTotalPixel ++;
            }
        }
    }
*/

    _ColorHistogram.FeedFrame( _pImageYCrCb, _pImageHandMask );

    _fLearned = true;
}

void HandRegion::LearnColor( IplImage * srcImage, IplImage * maskImage )
{
    if ( !srcImage || !maskImage )
    {
        _fLearned = false;
        return;
    }

    // Learn Histogram
    if ( !_pImage ) return;
    for ( int i = 0 ; i < 16 ; i ++ )
        for ( int j = 0 ; j < 16 ; j ++ )
            for ( int k = 0 ; k < 16 ; k ++ )
                _histPixel[i][j][k] = 0;
    _nTotalPixel = 0;
    for ( int i = 0 ; i < maskImage->width ; i ++ )
    {
        for ( int j = 0 ; j < maskImage->height ; j ++ )
        {
            if ( cvGetReal2D( maskImage, j, i ) == 255 )
            {
                unsigned char B = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 0 ];
                unsigned char G = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 1 ];
                unsigned char R = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 2 ];

                _histPixel[R/16][G/16][B/16] ++;
                _nTotalPixel ++;
            }
        }
    }
/*
    float samples[10000][3];
    int   nSamples = 0;

    //
    // Make Samples
    //
    for ( int i = 0 ; i < 320 ; i ++ )
    {
        for ( int j = 0 ; j < 240 ; j ++ )
        {
            if ( cvGetReal2D( _pImageHandMask, j, i ) > 0 )
            {
                unsigned char B = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 0 ];
                unsigned char G = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 1 ];
                unsigned char R = srcImage->imageData[ j * srcImage->widthStep + i * 3 + 2 ];

                samples[nSamples][0] = B;
                samples[nSamples][1] = G;
                samples[nSamples][2] = R;
                nSamples ++;
                if ( nSamples == 10000 ) break;
            }
        }
        if ( nSamples == 10000 ) break;
    }
    CvMat sampleMat = cvMat( nSamples, 3, CV_32FC1, samples );

    //
    // Train
    //
    _emModel.clear();
    _emModel.train( &sampleMat, 0, _emParams, 0 );
*/
    _fLearned = true;
}

void HandRegion::DrawContour( IplImage * dstImage, CvPoint start, IplImage * distImage )
{
    CvPoint current, prev;
    static int dir[8][2] = {{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1}};

    //
    // Check validity
    //
    if ( _pImage == 0 || dstImage == 0 ||
         start.x < 0 || start.x >= _pImage->width ||
         start.y < 0 || start.y >= _pImage->height )
    {
        return;
    }

    //
    // First, drop the start point
    //
    for ( ; start.y > 2 &&
            cvGetReal2D( _pImage, start.y - 1, start.x ) > 0 ;
            start.y = start.y - 1 );

    //
    // Traverse the contour
    //
    int nContour = 0;
    for ( prev = current = start ; ; )
    {
        //
        // Choose direction
        for ( int i = 0 ; i < 8 ; i ++ )
        {
            int newX = current.x + dir[i][0];
            int newY = current.y + dir[i][1];

            if ( newX > 0 && newX < _pImage->width-1 &&
                 newY > 0 && newY < _pImage->height-1 &&
                 ( newX != current.x || newY != current.y ) &&
                 ( newX != prev.x || newY != prev.y ) &&
                 cvGetReal2D( _pImage, newY, newX ) > 0 )
            {
                bool fEdge = false;
                for ( int j = 0 ; j < 8 ; j ++ )
                {
                    int neighborX = newX + dir[j][0];
                    int neighborY = newY + dir[j][1];
                    if ( cvGetReal2D( _pImage, neighborY, neighborX ) == 0 )
                    {
                        fEdge = true;
                        break;
                    }
                }

                if ( fEdge )
                {
                    //
                    // Draw the contour point
                    //
                    cvCircle( dstImage, current, 1, CV_RGB(0,255,0), 1, 8, 0 );
    //                cvSet2D( dstImage, current.y, current.x, CV_RGB(0,255,0) );

                    //
                    // Move the point
                    //
                    prev = current;
                    current.x = newX;
                    current.y = newY;
                    break;
                }
            }
        }

        if ( current.x == start.x && current.y == start.y )
        {
            break;
        }

        nContour ++;
        if ( nContour == 10000 ) break;
    }
}
