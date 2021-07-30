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

#ifndef ELLIPSE_POINT_H
#define ELLIPSE_POINT_H

//#include "precomp.h"
#include <opencv2/core/core.hpp>


namespace cv
{

namespace runetag
{

class EllipseFitter;
class EllipsePoint 
{
public:
    friend class SlotFitter;
    cv::Matx33d ellipse_norm;
    cv::Matx33d ellipse;

private:
    cv::RotatedRect er;
    cv::Matx33d VR1;
    cv::Matx33d VR2;
    bool unassigned;
    bool slotted;
    double L2inv;

    mutable cv::Point2d centerCache;
    mutable cv::Point2d centerNormPlaneCache;
    mutable bool cacheValidCenter;
    mutable bool cacheValidCenterNormPlane;

    // -------------
    // AUX OPERATION
    // -------------

    /*
    Set either VR1 or VR2
    Determines the setting based on the boolean input parameter
    */
    void findOrientation(const cv::Matx33d& V, double g, double h, double d1, double d2, bool& orientation1Found, bool& orientation2Found);
    /*
    Builds the rotational matrix
    */
    static void buildR(double g, double h, double s1, double s2, cv::Matx33d& outMatrix);
    /*
    Checks if the ellipse normal transformed has the right orientation
    */
    static bool checkN(const cv::Matx33d& VR);

public: 
    EllipsePoint( ) {};
    EllipsePoint( cv::RotatedRect _ellipse, const double cx, const double cy );
    EllipsePoint( const EllipsePoint& copy );
    const EllipsePoint& operator=( const EllipsePoint& other );

	inline cv::RotatedRect get_as_rotated_rect()
	{
		return er;
	}

    inline bool isAssigned() const
    {
        return !unassigned;
    }
    
    inline bool isSlotted() const
    {
        return slotted;
    }
    
    inline const cv::Matx33d& getVR1() const
    {
        return VR1;
    }

    inline const cv::Matx33d& getVR2() const 
    { 
        return VR2;
    }
    
    inline const cv::Matx33d& getEllipse() const 
    { 
        return ellipse; 
    }
    
    inline const cv::Matx33d& getEllipseNorm() const 
    { 
        return ellipse_norm; 
    }
    
    inline double getL2inv() const 
    {
        return L2inv;
    }

    void setAssigned() 
    { 
        unassigned=false; 
    }

    inline bool isinside( float px, float py)
    {
        //Layer boundary
        const double& x = px;
        const double& y = py;
        const double& A_max = ellipse(0, 0);
        const double& B_max = ellipse(0, 1);
        const double& C_max = ellipse(1, 1);
        const double& D_max = ellipse(0, 2);
        const double& E_max = ellipse(1, 2);
        const double& F_max = ellipse(2, 2);

        return (A_max * x * x + 2.0 * B_max * x * y + C_max * y * y + 2.0 * D_max * x + 2.0 * E_max * y + F_max < 0.0) ;        
    }

    // Main operation
	bool refine( const cv::Mat& gradient_x, const cv::Mat& gradient_y, const cv::Mat& intrinsics, cv::Mat dbg_img );
    void toCircle( const cv::Matx33d& VR );
    void calcVR( const double f );

    cv::Point2d getCenterOnNormPlane( ) const;
    cv::Point2d getCenter( ) const;

};

} // namespace runetag
} // namespace cv

#endif
