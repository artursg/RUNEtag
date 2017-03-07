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

#ifndef SLOT_FITTER_H
#define SLOT_FITTER_H

//#include "precomp.hpp"
#include "ellipsepoint.hpp"
#include "slot.hpp"
#include "markerdetected.hpp"
#include <set>
#include <vector>

namespace cv
{
namespace runetag
{

#define FIT_CACHE_SIZE 16

class SlotFitter 
{
private:
    cv::Matx33d VR;
    mutable cv::Matx33d refit;

    cv::Mat intrinsics;
    
    std::vector< EllipsePoint > auto_inliers;
    std::vector< EllipsePoint >& ellipses;
    EllipsePoint refined;

    bool valid;
    int _min_symbols_for_layer;

    std::vector< cv::Point2f > points_coords;
    std::vector< cv::Point2f > fit_slots;
    std::vector< std::vector<cv::Point2f> > fit_slots_centers;
    cv::Point2f slots_center;

    mutable cv::Matx33d fit_cache[FIT_CACHE_SIZE];
    mutable bool fit_cache_exists[FIT_CACHE_SIZE];

    cv::Matx33d getFitWithOffset( int currLayer, unsigned int num_layers, double radius ) const;
    bool buildCodeForLayer( int layer, int num_layers, double rSqr, std::vector< Slot >& code ) const;

    static bool ellipseContains( const cv::Matx33d& ellipse, const cv::Point2d& p );
    static cv::Point2d transformPointFromCircleToEllipse( cv::Point2d p, const cv::Matx33d& VR, double f );


public:

    SlotFitter( const cv::Matx33d& _VR, std::vector<EllipsePoint>& _ellipses, const cv::Matx33d& fit_min, const cv::Matx33d& fit_max, const cv::Mat& _intrinsics, int min_support = 6, int min_symbols_for_layer = 3  );
    
    void fit( std::vector< EllipsePoint >& ring_ellipses, double radius_ratio, double gap_factor, int num_layers, std::vector<MarkerDetected>& possible_markers );
    void fit( double radius_ratio, double gap_factor, int num_layers, std::vector<MarkerDetected>& possible_markers );

    inline bool isValid() const  
    { 
        return valid; 
    }
};

} // namespace runetag
} // namespace cv

#endif
