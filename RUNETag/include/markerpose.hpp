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

#ifndef _MARKER_POSE_H
#define _MARKER_POSE_H

//#include "precomp.hpp"
#include <opencv2/opencv.hpp>
#include "markerdetected.hpp"


namespace cv
{
namespace runetag
{
    const unsigned int FLAG_REPROJ_ERROR = 1;
    const unsigned int FLAG_REFINE = 2;

    /// <summary> Camera pose as a transformation from camera coordinates to world coordinates </summary>
    struct Pose 
    {
        /// <value> Rotation matrix </value>
        cv::Mat R;
        /// <value> Translation vector </value>
        cv::Mat t;
    };

    extern Pose findPose( const MarkerDetected& detected, const cv::Mat& intrinsics, const cv::Mat& distortion, bool* pose_ok = 0, unsigned int method=SOLVEPNP_ITERATIVE, unsigned int flag = 0 );

} // namespace runetag
} // namespace cv

#endif
