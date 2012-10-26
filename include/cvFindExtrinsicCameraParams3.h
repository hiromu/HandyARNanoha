/**
 *
 * File:
 *          cvFindExtrinsicCameraParams3.h
 *
 * Description:
 *          Find Extrinsic Camera Parameters with more reliable result.
 *          Check if the preliminary result is too much different than
 *          the previous parameters. If so, force the refinement step
 *          starts from the previous parameters, ignoring new results.
 *
 *
 * Author:
 *          Taehee Lee
 *
 * Date:
 *          02/23/2007
 *
 */

#ifndef _CV_FIND_EXTRINSIC_CAMERA_PARAMS3_H_
#define _CV_FIND_EXTRINSIC_CAMERA_PARAMS3_H_

#include <opencv/cv.h>

bool
cvFindExtrinsicCameraParams3( const CvMat* obj_points,
                  const CvMat* img_points, const CvMat* A,
                  const CvMat* dist_coeffs,
                  CvMat* r_vec, CvMat* t_vec );

#endif // _CV_FIND_EXTRINSIC_CAMERA_PARAMS3_H_
