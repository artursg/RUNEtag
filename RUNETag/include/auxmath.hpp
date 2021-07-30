/**
* RUNETag fiducial markers library
*
* -----------------------------------------------------------------------------
* The MIT License (MIT)
* 
* Copyright (c) 2015 Filippo Bergamasco 
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
*/

#ifndef AUX_MATH_HPP
#define AUX_MATH_HPP

//#include "precomp.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace cv {
namespace runetag {


template <typename T>
inline bool almost_zero(T a, double e)
{
    return (a == T(0)) || (a > 0 && a < e) || (a < 0 && a > -e);
}

// VideoDemo utility
extern double TDelta( const cv::Mat& T1, const cv::Mat& T2 );

// VideoDemo utility
extern double RDelta( const cv::Mat& R1, const cv::Mat& R2 );

// VideoDemo - AuxRenderer utility
cv::RotatedRect ellipseToRotatedRect( const cv::Matx33d& ellipse );

// EllipseFitter - SlotFitter utility
extern cv::Matx33d transformToEllipse( const cv::Matx33d& Qc, const cv::Matx33d& VR, double k );

// EllipseFitter - EllipsePoint - EllipseRefine - MarkerPose - SlotFitter utility
extern cv::Point2d ellipseCenter( const cv::Matx33d& e );

// SlotFitter utility (nella parte commentata)
extern double ellipseRadius( const cv::Matx33d& Qc );

// SlotFitter utility
extern cv::Point2d transformPointFromCircleToEllipse( cv::Point2d p, const cv::Matx33d& VR, double f );

extern cv::Matx33d& scalarDivision( cv::Matx33d& targetMatrix, double scalar );

}
}

#endif
