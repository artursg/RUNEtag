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

#ifndef MARKER_DETECTOR_H
#define MARKER_DETECTOR_H

//#include "precomp.hpp"
#include <opencv2/core/core.hpp>
#include "ellipsepoint.hpp"
#include "markerdetected.hpp"
#include "digitalmarkermodel.hpp"
#include "slotfitter.hpp"
#include <vector>

namespace cv 
{
namespace runetag
{

struct DetectorStats 
{
    unsigned int num_pairs_checked;         //Number of total pairs checked
    unsigned int num_pairs_fitted;          //Number of total pairs for which two big ellipses can be fitted
    unsigned int num_pairs_not_fitted;      //Number of total pairs for which two big ellipses cannot be fitted
    unsigned int num_pairs_not_refitted;    //Number of total pairs for which two big ellipses cannot be refitted (too few points to re-fit a circle in least-square sense)
    unsigned int num_models_tested;         //Number of candidate marker which has been tested against a known model

    DetectorStats() : num_pairs_checked(0), num_pairs_fitted(0), num_pairs_not_fitted(0),
            num_pairs_not_refitted(0), num_models_tested(0) {}
};

class MarkerDetector 
{

private:

    cv::Mat intrinsics;
    std::map< long, DigitalMarkerModel > models;
    std::vector<EllipsePoint> ellipsePoints;
    unsigned int min_pts_for_level;

    void toEllipsePoints( const std::vector<cv::RotatedRect>& ellipses );
    bool findModel( const std::vector<MarkerDetected>& possible_markers, std::vector<MarkerDetected>& markers_detected );
    bool tryFit( SlotFitter& sf, std::map<int, MarkerDetected>& markers_by_id );

    

public:
    cv::Mat dbgimage;

    MarkerDetector( const cv::Mat& _intrinsics );
    virtual ~MarkerDetector();

    int addModelsFromFile( std::string filename );

    int detectMarkers( const std::vector<cv::RotatedRect>& ellipses, std::vector<MarkerDetected>& markers_detected );
    int detectMarkers( const std::vector<cv::RotatedRect>& ellipses, std::vector<MarkerDetected>& markers_detected, DetectorStats& stats );

    class MarkerDetectorException : public std::runtime_error 
    {
    public:
        MarkerDetectorException( std::string reason ) : std::runtime_error(reason.c_str()) {}
    };
};

} // namespace runetag
} // namespace cv

#endif
