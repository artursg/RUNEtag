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
#ifndef _AUX_RENDERER_H
#define _AUX_RENDERER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "markerpose.hpp"
#include <sstream>


namespace cv {
namespace runetag {

class EllipsePoint;
class Slot;
class MarkerDetected;

namespace AuxRenderer {

    template <typename T>
    std::string toStr( T val )
    {
        std::stringstream ss;
        ss << val;
        return ss.str();
    }

	extern void drawEllipsePoint( cv::Mat frame, const EllipsePoint& e, const cv::Mat& intrinsics, cv::Scalar color );
	extern void drawEllipse( cv::Mat& frame, const cv::RotatedRect& el, const cv::Mat& intrinsics );
    extern void drawEllipse( cv::Mat frame, const cv::Mat& e, const cv::Mat& intrinsics, cv::Scalar color );
    extern void fillEllipse( cv::Mat frame, const cv::Mat& e, const cv::Mat& intrinsics, cv::Scalar color );
	extern void drawPoint( cv::Mat& frame, const cv::Point2f p, const cv::Mat& intrinsics, cv::Scalar color );
	extern void drawPoint( cv::Mat& frame, const cv::Point2f p, const cv::Mat& intrinsics );
	extern void drawVector( cv::Mat& frame, const cv::Point2f center, const cv::Point2f vector, cv::Scalar color, const cv::Mat& intrinsics );
	extern void drawLine( cv::Mat& frame, const cv::Point2f p1, const cv::Point2f p2, cv::Scalar color, const cv::Mat& intrinsics );

	extern void fillSlot( cv::Mat& frame, const Slot& slot, cv::Scalar color, const cv::Mat& intrinsics );

    extern void drawDetectedMarker( cv::Mat& frame, const MarkerDetected& m, const cv::Mat& intrinsics );

    extern void drawDetectedMarker3D( cv::Mat& frame, const MarkerDetected& m, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion, cv::Scalar color = CV_RGB(0,0,255) );

    extern void drawDetectedMarker3DCylinder( cv::Mat& frame, const MarkerDetected& m, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion, cv::Scalar color );

    extern void drawDetectedMarker3DCylinder2( cv::Mat& frame, const MarkerDetected& m, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion );

    // CVPR
    extern void drawDetectedMarker3DSlots( cv::Mat& frame, const MarkerDetected& m, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion, cv::Scalar color = CV_RGB(0,0,255) );
    extern void drawDetectedMarker3Dfits( cv::Mat& frame, const MarkerDetected& m, unsigned int slot1, unsigned int slot2, unsigned int real_slot1, unsigned int real_slot2, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion );

}

}
}

#endif
