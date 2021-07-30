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

#ifndef ELLIPSE_DETECTOR_H
#define ELLIPSE_DETECTOR_H

//#include "precomp.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>


namespace cv 
{

namespace runetag 
{


/// <summary> Image ellipses detector</summary>
///	<remarks>
///	This class is primary used to detect all ellipses inside an image to be
/// further processed by ElliMarkDetector.<br/>
/// The actual ellipse detection process involves in the following steps:<br/>
/// 1) Conversion to grayscale<br/>
/// 2) Thresholding to a black/white image<br/>
/// 3) Contour detection <br/>
/// 4) Ellipse fitting to each contour <br/>
/// 5) Detected ellipses filtering (based on min/max area, min roundness, etc)
/// </remarks>
///
class EllipseDetector 
{

private:

    unsigned short max_ellipse_contour_points;
    unsigned short min_ellipse_contour_points;
    float min_ellipse_area;
    float max_ellipse_area;
    float min_required_roundness;
    float max_mse;
    float size_compensation;

    EllipseDetector( const EllipseDetector& copy );
    EllipseDetector& operator=( const EllipseDetector& other );

public:
    /// <summary> Class constructor </summary>
    /// <param name="_min_ellipse_contour_points"> Minimum number of contour points required to fit an ellipse </param>
    /// <param name="_max_ellipse_contour_points"> Maximum number of contour points allowed to fit an ellipse </param>
    /// <param name="_min_ellipse_area"> Minumum area allowed for a detected ellipse </param>
    /// <param name="_max_ellipse_area"> Maximum area allowed for a detected ellipse </param>
    /// <param name="_min_required_roundness"> Minimum width/height ratio allowed for a detected ellipse </param>
    /// <param name="_max_mse"> Maximum fitting error for a detected ellipse </param>
    ///
    EllipseDetector( unsigned short _min_ellipse_contour_points = 10,
                     unsigned short _max_ellipse_contour_points = 300, 
                     float _min_ellipse_area = 10.0f,
                     float _max_ellipse_area = 1000.0f,
                     float _min_required_roundness = 0.3f,
                     float _max_mse = 0.24f,
                     float _size_compensation = -1.5) :     min_ellipse_contour_points( _min_ellipse_contour_points ),
                                                            max_ellipse_contour_points( _max_ellipse_contour_points ),
                                                            min_ellipse_area( _min_ellipse_area ),
                                                            max_ellipse_area( _max_ellipse_area ),
                                                            min_required_roundness( _min_required_roundness ),
                                                            max_mse( _max_mse ),
                                                            size_compensation(_size_compensation)
                                                    {  }
    ~EllipseDetector(void) {};

    /// <summary> Detects ellipses inside an image</summary>
    /// <param name="frame"> Input image </param>
    /// <param name="detected"> Output vector to append all detected ellipses </param>
    /// <returns> Number of ellipses detected </returns>
    unsigned int detectEllipses( const cv::Mat& frame, std::vector< cv::RotatedRect >& detected );

};

} // namespace runetag
} // namespace cv

#endif
