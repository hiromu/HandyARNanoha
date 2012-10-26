#include "cvFindExtrinsicCameraParams3.h"
#include <stdio.h>

bool
cvFindExtrinsicCameraParams3( const CvMat* obj_points,
                  const CvMat* img_points, const CvMat* A,
                  const CvMat* dist_coeffs,
                  CvMat* r_vec, CvMat* t_vec )
{
    bool fGood = true;

    const int max_iter = 20;
    CvMat *_M = 0, *_Mxy = 0, *_m = 0, *_mn = 0, *_L = 0, *_J = 0;
    
    CV_FUNCNAME( "cvFindExtrinsicCameraParams3" );

    __BEGIN__;

    int i, j, count;
    double a[9], k[4] = { 0, 0, 0, 0 }, R[9], ifx, ify, cx, cy;
    double Mc[3] = {0, 0, 0}, MM[9], U[9], V[9], W[3];
    double JtJ[6*6], JtErr[6], JtJW[6], JtJV[6*6], delta[6], param[6];
    CvPoint3D64f* M = 0;
    CvPoint2D64f *m = 0, *mn = 0;
    CvMat _a = cvMat( 3, 3, CV_64F, a );
    CvMat _R = cvMat( 3, 3, CV_64F, R );
    CvMat _r = cvMat( 3, 1, CV_64F, param );
    CvMat _t = cvMat( 3, 1, CV_64F, param + 3 );
    CvMat _Mc = cvMat( 1, 3, CV_64F, Mc );
    CvMat _MM = cvMat( 3, 3, CV_64F, MM );
    CvMat _U = cvMat( 3, 3, CV_64F, U );
    CvMat _V = cvMat( 3, 3, CV_64F, V );
    CvMat _W = cvMat( 3, 1, CV_64F, W );
    CvMat _JtJ = cvMat( 6, 6, CV_64F, JtJ );
    CvMat _JtErr = cvMat( 6, 1, CV_64F, JtErr );
    CvMat _JtJW = cvMat( 6, 1, CV_64F, JtJW );
    CvMat _JtJV = cvMat( 6, 6, CV_64F, JtJV );
    CvMat _delta = cvMat( 6, 1, CV_64F, delta );
    CvMat _param = cvMat( 6, 1, CV_64F, param );
    CvMat _dpdr, _dpdt;

    if( !CV_IS_MAT(obj_points) || !CV_IS_MAT(img_points) ||
        !CV_IS_MAT(A) || !CV_IS_MAT(r_vec) || !CV_IS_MAT(t_vec) )
        CV_ERROR( CV_StsBadArg, "One of required arguments is not a valid matrix" );

    count = MAX(obj_points->cols, obj_points->rows);
    CV_CALL( _M = cvCreateMat( 1, count, CV_64FC3 ));
    CV_CALL( _Mxy = cvCreateMat( 1, count, CV_64FC2 ));
    CV_CALL( _m = cvCreateMat( 1, count, CV_64FC2 ));
    CV_CALL( _mn = cvCreateMat( 1, count, CV_64FC2 ));
    M = (CvPoint3D64f*)_M->data.db;
    m = (CvPoint2D64f*)_m->data.db;
    mn = (CvPoint2D64f*)_mn->data.db;

    CV_CALL( cvConvertPointsHomogenious( obj_points, _M ));
    CV_CALL( cvConvertPointsHomogenious( img_points, _m ));
    CV_CALL( cvConvert( A, &_a ));

    if( dist_coeffs )
    {
        CvMat _k;
        if( !CV_IS_MAT(dist_coeffs) ||
            CV_MAT_DEPTH(dist_coeffs->type) != CV_64F &&
            CV_MAT_DEPTH(dist_coeffs->type) != CV_32F ||
            dist_coeffs->rows != 1 && dist_coeffs->cols != 1 ||
            dist_coeffs->rows*dist_coeffs->cols*CV_MAT_CN(dist_coeffs->type) != 4 )
            CV_ERROR( CV_StsBadArg,
                "Distortion coefficients must be 1x4 or 4x1 floating-point vector" );

        _k = cvMat( dist_coeffs->rows, dist_coeffs->cols,
                    CV_MAKETYPE(CV_64F,CV_MAT_CN(dist_coeffs->type)), k );
        CV_CALL( cvConvert( dist_coeffs, &_k ));
    }

    if( CV_MAT_DEPTH(r_vec->type) != CV_64F && CV_MAT_DEPTH(r_vec->type) != CV_32F ||
        r_vec->rows != 1 && r_vec->cols != 1 ||
        r_vec->rows*r_vec->cols*CV_MAT_CN(r_vec->type) != 3 )
        CV_ERROR( CV_StsBadArg, "Rotation vector must be 1x3 or 3x1 floating-point vector" );

    if( CV_MAT_DEPTH(t_vec->type) != CV_64F && CV_MAT_DEPTH(t_vec->type) != CV_32F ||
        t_vec->rows != 1 && t_vec->cols != 1 ||
        t_vec->rows*t_vec->cols*CV_MAT_CN(t_vec->type) != 3 )
        CV_ERROR( CV_StsBadArg,
            "Translation vector must be 1x3 or 3x1 floating-point vector" );

    ifx = 1./a[0]; ify = 1./a[4];
    cx = a[2]; cy = a[5];

    // normalize image points
    // (unapply the intrinsic matrix transformation and distortion)
    for( i = 0; i < count; i++ )
    {
        double x = (m[i].x - cx)*ifx, y = (m[i].y - cy)*ify, x0 = x, y0 = y;

        // compensate distortion iteratively
        if( dist_coeffs )
            for( j = 0; j < 5; j++ )
            {
                double r2 = x*x + y*y;
                double icdist = 1./(1 + k[0]*r2 + k[1]*r2*r2);
                double delta_x = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
                double delta_y = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
                x = (x0 - delta_x)*icdist;
                y = (y0 - delta_y)*icdist;
            }
        mn[i].x = x; mn[i].y = y;

        // calc mean(M)
        Mc[0] += M[i].x;
        Mc[1] += M[i].y;
        Mc[2] += M[i].z;
    }

    Mc[0] /= count;
    Mc[1] /= count;
    Mc[2] /= count;

    cvReshape( _M, _M, 1, count );
    cvMulTransposed( _M, &_MM, 1, &_Mc );
    cvSVD( &_MM, &_W, 0, &_V, CV_SVD_MODIFY_A + CV_SVD_V_T );

    // initialize extrinsic parameters
    if( W[2]/W[1] < 1e-3 || count < 4 )
    {
        // a planar structure case (all M's lie in the same plane)
        double tt[3], h[9], h1_norm, h2_norm;
        CvMat* R_transform = &_V;
        CvMat T_transform = cvMat( 3, 1, CV_64F, tt );
        CvMat _H = cvMat( 3, 3, CV_64F, h );
        CvMat _h1, _h2, _h3;

        if( V[2]*V[2] + V[5]*V[5] < 1e-10 )
            cvSetIdentity( R_transform );

        if( cvDet(R_transform) < 0 )
            cvScale( R_transform, R_transform, -1 );

        cvGEMM( R_transform, &_Mc, -1, 0, 0, &T_transform, CV_GEMM_B_T );

        for( i = 0; i < count; i++ )
        {
            const double* Rp = R_transform->data.db;
            const double* Tp = T_transform.data.db;
            const double* src = _M->data.db + i*3;
            double* dst = _Mxy->data.db + i*2;

            dst[0] = Rp[0]*src[0] + Rp[1]*src[1] + Rp[2]*src[2] + Tp[0];
            dst[1] = Rp[3]*src[0] + Rp[4]*src[1] + Rp[5]*src[2] + Tp[1];
        }

        cvFindHomography( _Mxy, _mn, &_H );

        cvGetCol( &_H, &_h1, 0 );
        _h2 = _h1; _h2.data.db++;
        _h3 = _h2; _h3.data.db++;
        h1_norm = sqrt(h[0]*h[0] + h[3]*h[3] + h[6]*h[6]);
        h2_norm = sqrt(h[1]*h[1] + h[4]*h[4] + h[7]*h[7]);

        cvScale( &_h1, &_h1, 1./h1_norm );
        cvScale( &_h2, &_h2, 1./h2_norm );
        cvScale( &_h3, &_t, 2./(h1_norm + h2_norm));
        cvCrossProduct( &_h1, &_h2, &_h3 );

        cvRodrigues2( &_H, &_r );
        cvRodrigues2( &_r, &_H );
        cvMatMulAdd( &_H, &T_transform, &_t, &_t );
        cvMatMul( &_H, R_transform, &_R );
        cvRodrigues2( &_R, &_r );
    }
    else
    {
        // non-planar structure. Use DLT method
        double* L;
        double LL[12*12], LW[12], LV[12*12], sc;
        CvMat _LL = cvMat( 12, 12, CV_64F, LL );
        CvMat _LW = cvMat( 12, 1, CV_64F, LW );
        CvMat _LV = cvMat( 12, 12, CV_64F, LV );
        CvMat _RRt, _RR, _tt;

        CV_CALL( _L = cvCreateMat( 2*count, 12, CV_64F ));
        L = _L->data.db;

        for( i = 0; i < count; i++, L += 24 )
        {
            double x = -mn[i].x, y = -mn[i].y;
            L[0] = L[16] = M[i].x;
            L[1] = L[17] = M[i].y;
            L[2] = L[18] = M[i].z;
            L[3] = L[19] = 1.;
            L[4] = L[5] = L[6] = L[7] = 0.;
            L[12] = L[13] = L[14] = L[15] = 0.;
            L[8] = x*M[i].x;
            L[9] = x*M[i].y;
            L[10] = x*M[i].z;
            L[11] = x;
            L[20] = y*M[i].x;
            L[21] = y*M[i].y;
            L[22] = y*M[i].z;
            L[23] = y;
        }

        cvMulTransposed( _L, &_LL, 1 );
        cvSVD( &_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T );
        _RRt = cvMat( 3, 4, CV_64F, LV + 11*12 );
        cvGetCols( &_RRt, &_RR, 0, 3 );
        cvGetCol( &_RRt, &_tt, 3 );
        if( cvDet(&_RR) < 0 )
            cvScale( &_RRt, &_RRt, -1 );
        sc = cvNorm(&_RR);
        cvSVD( &_RR, &_W, &_U, &_V, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T );
        cvGEMM( &_U, &_V, 1, 0, 0, &_R, CV_GEMM_A_T );
        cvScale( &_tt, &_t, cvNorm(&_R)/sc );
        cvRodrigues2( &_R, &_r );
        cvReleaseMat( &_L );
    }

    //
    // Check if new r and t are good
    //
    if ( cvGetReal1D( r_vec, 0 ) ||
         cvGetReal1D( r_vec, 1 ) ||
         cvGetReal1D( r_vec, 2 ) ||
         cvGetReal1D( t_vec, 0 ) ||
         cvGetReal1D( t_vec, 1 ) ||
         cvGetReal1D( t_vec, 2 ) )
    {
        //
        // perfom this only when the old r and t exist.
        //
        CvMat * R_inv = cvCreateMat( 3, 3, CV_64FC1 );
        CvMat * r_old = cvCreateMat( 3, 1, CV_64FC1 );
        CvMat * R_old = cvCreateMat( 3, 3, CV_64FC1 );
        CvMat * t_old = cvCreateMat( 3, 1, CV_64FC1 );
        // get new center
        cvInvert( &_R, R_inv );
        double new_center[3];
        CvMat newCenter = cvMat( 3, 1, CV_64FC1, new_center );
        cvMatMul( R_inv, &_t, &newCenter );
        cvScale( &newCenter, &newCenter, -1 );
        // get old center
        cvConvert( r_vec, r_old );
        cvConvert( t_vec, t_old );
        cvRodrigues2( r_old, R_old );
        cvInvert( R_old, R_inv );
        double old_center[3];
        CvMat oldCenter = cvMat( 3, 1, CV_64FC1, old_center );
        cvMatMul( R_inv, t_old, &oldCenter );
        cvScale( &oldCenter, &oldCenter, -1 );
        // get difference
        double diff_center = 0;
        for ( int i = 0; i < 3 ; i ++ )
            diff_center += pow( new_center[i] - old_center[i], 2 );
        diff_center = sqrt( diff_center );
        if ( diff_center > 300 )
        {
            printf("diff_center = %.2f --> set initial r and t as same as  the previous\n", diff_center);
            cvConvert(r_vec, &_r);
            cvConvert(t_vec, &_t);
            fGood = false;
        }
//        else printf("diff_center = %.2f\n", diff_center );
        
        cvReleaseMat( &R_inv );
        cvReleaseMat( &r_old );
        cvReleaseMat( &R_old );
        cvReleaseMat( &t_old );
    }
    
    if ( fGood )
    {
        CV_CALL( _J = cvCreateMat( 2*count, 6, CV_64FC1 ));
        cvGetCols( _J, &_dpdr, 0, 3 );
        cvGetCols( _J, &_dpdt, 3, 6 );

        // refine extrinsic parameters using iterative algorithm
        for( i = 0; i < max_iter; i++ )
        {
            double n1, n2;
            cvReshape( _mn, _mn, 2, 1 );
            cvProjectPoints2( _M, &_r, &_t, &_a, dist_coeffs,
                              _mn, &_dpdr, &_dpdt, 0, 0, 0 );
            cvSub( _m, _mn, _mn );
            cvReshape( _mn, _mn, 1, 2*count );

            cvMulTransposed( _J, &_JtJ, 1 );
            cvGEMM( _J, _mn, 1, 0, 0, &_JtErr, CV_GEMM_A_T );
            cvSVD( &_JtJ, &_JtJW, 0, &_JtJV, CV_SVD_MODIFY_A + CV_SVD_V_T );
            if( JtJW[5]/JtJW[0] < 1e-12 )
                break;
            cvSVBkSb( &_JtJW, &_JtJV, &_JtJV, &_JtErr,
                      &_delta, CV_SVD_U_T + CV_SVD_V_T );
            cvAdd( &_delta, &_param, &_param );
            n1 = cvNorm( &_delta );
            n2 = cvNorm( &_param );
            if( n1/n2 < 1e-10 )
                break;
        }

        _r = cvMat( r_vec->rows, r_vec->cols,
            CV_MAKETYPE(CV_64F,CV_MAT_CN(r_vec->type)), param );
        _t = cvMat( t_vec->rows, t_vec->cols,
            CV_MAKETYPE(CV_64F,CV_MAT_CN(t_vec->type)), param + 3 );

        cvConvert( &_r, r_vec );
        cvConvert( &_t, t_vec );
    }

    __END__;

    cvReleaseMat( &_M );
    cvReleaseMat( &_Mxy );
    cvReleaseMat( &_m );
    cvReleaseMat( &_mn );
    cvReleaseMat( &_L );
    cvReleaseMat( &_J );

    return fGood;
}
