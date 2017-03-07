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

//#include "precomp.hpp"
#include "ellipsepoint.hpp"
#include "ellipserefine.hpp"
#include "auxmath.hpp"
#include <iostream>

using namespace cv::runetag;

EllipsePoint::EllipsePoint( cv::RotatedRect _ellipse, const double cx, const double cy ) : er(_ellipse), 
    unassigned(true), 
    cacheValidCenter(false), 
    cacheValidCenterNormPlane(false) 
{
    _ellipse.center.x -= static_cast<float>(cx);//cx
    _ellipse.center.y -= static_cast<float>(cy);//cy

    cv::Matx22d Rinv;
    cv::Matx21d Tinv;

    const double th = _ellipse.angle*CV_PI/180.;

    //Build Rinv: rotation matrix
    Rinv(0,0) = cos( th );
    Rinv(0,1) = -sin( th );
    Rinv(1,0) = sin( th );
    Rinv(1,1) = cos( th );

    //Build Tinv: translation matrix
    Tinv(0,0) = -_ellipse.center.x;
    Tinv(1,0) = -_ellipse.center.y;

    Tinv = Rinv*Tinv;

    double a = _ellipse.size.width/2.0;
    double b = _ellipse.size.height/2.0;

    //cv::Mat Qcan(3, 3, CV_64F, cv::Scalar(0.0) );
    cv::Matx33d Qcan;
    Qcan.zeros();

    //Build Qcan
    Qcan(0,0)= 1.0/(a*a);
    Qcan(1,1)= 1.0/(b*b);
    Qcan(2,2)= -1.0;

    cv::Matx33d RTinv;

    //Build RTinv
    RTinv(0,0) = Rinv(0,0);
    RTinv(0,1) = Rinv(0,1);
    RTinv(0,2) = Tinv(0,0);

    RTinv(1,0) = Rinv(1,0);
    RTinv(1,1) = Rinv(1,1);
    RTinv(1,2) = Tinv(1,0);

    RTinv(2,0) = 0.0;
    RTinv(2,1) = 0.0;
    RTinv(2,2) = 1.0;
    
    ellipse = (RTinv.t()*Qcan*RTinv);
}


EllipsePoint::EllipsePoint( const EllipsePoint& copy ) 
{
    VR1 = copy.VR1;
    VR2 = copy.VR2;
    ellipse = copy.ellipse;
    ellipse_norm = copy.ellipse_norm;
    L2inv = copy.L2inv;
    unassigned = copy.unassigned;
    cacheValidCenter = copy.cacheValidCenter;
    cacheValidCenterNormPlane = copy.cacheValidCenterNormPlane;
    er = copy.er;
}


const EllipsePoint& EllipsePoint::operator=( const EllipsePoint& other ) 
{
    if( &other == this )
    {
        return *this;
    }

    VR1 = other.VR1;
    VR2 = other.VR2;
    ellipse = other.ellipse;
    ellipse_norm = other.ellipse_norm;
    L2inv = other.L2inv;
    unassigned = other.unassigned;
    cacheValidCenter = other.cacheValidCenter;
    cacheValidCenterNormPlane = other.cacheValidCenterNormPlane;
    er = other.er;

    return *this;
}


bool EllipsePoint::refine( const cv::Mat& gradient_x, const cv::Mat& gradient_y, const cv::Mat& intrinsics, cv::Mat dbg_img  ) 
{
	cv::Matx33d refined_ellipse;
    if( ellipserefine( er, gradient_x, gradient_y, intrinsics.at<double>(0,2), intrinsics.at<double>(1,2), refined_ellipse, dbg_img ) )
	{
		ellipse = refined_ellipse;
		cacheValidCenter = false;
		return true;
	}
	return false;
}


void EllipsePoint::toCircle( const cv::Matx33d& VR ) 
{
    cacheValidCenterNormPlane = false;
    cv::Matx33d C = VR.t()*ellipse_norm*VR;
    double nr = ( C(0,0) + C(1,1) ) / 2.0;
    scalarDivision(C, nr);
    ellipse_norm = C;

    //std::cout << "To circle: " << ellipse_norm(0,0) << " - " << ellipse_norm(1,1) << " ecc: " << ellipse_norm(0,0)/ellipse_norm(1,1) << std::endl;
    //std::cout << (cv::Mat)ellipse_norm << std::endl;
}


void EllipsePoint::calcVR( const double f ) 
{
    cacheValidCenter = cacheValidCenterNormPlane = false;

    ellipse_norm = ellipse;
    //Build Q
    ellipse_norm(0,2) /= f;
    ellipse_norm(1,2) /= f;
    ellipse_norm(2,0) /= f;
    ellipse_norm(2,1) /= f;
    ellipse_norm(2,2) /= f*f;

    //Convert Q to diagonal matrix
    cv::SVD svd;
    svd( ellipse_norm );
    cv::Matx33d V(svd.u);

    // svd.w == eigenvalues
    const double L1 = svd.w.at<double>(0,0);
    const double L2 = svd.w.at<double>(1,0);
    const double L3 = svd.w.at<double>(2,0);
    L2inv = 1.0/L2;

    //Calc g and h
    const double g = sqrt( (L2-L3)/(L1-L3) );
    const double h = sqrt( (L1-L2)/(L1-L3) );

    // Discards 2 of the 4 possibile orientation of the ellipse
    // that are specular with the others
    bool VR1_set=false;
    bool VR2_set=false;
    findOrientation(V, g, h, 1.0, 1.0, VR1_set, VR2_set);
    findOrientation(V, g, h, -1.0, 1.0, VR1_set, VR2_set);
    findOrientation(V, g, h, 1.0, -1.0, VR1_set, VR2_set);
    findOrientation(V, g, h, -1.0, -1.0, VR1_set, VR2_set);

}

void EllipsePoint::findOrientation(const cv::Matx33d& V, double g, double h, double dir1, double dir2, bool& orientation1Found, bool& orientation2Found) 
{

    if (!orientation2Found) 
    {
        cv::Matx33d R;
        buildR( g, h, dir1, dir2, R);
        cv::Matx33d VR(V*R);
        if( checkN(VR) ) 
        {
            if( orientation1Found ) 
            {
                VR2 = VR;
                orientation2Found = true;
            } 
            else 
            {
                VR1 = VR;
                orientation1Found = true;
            }
        }
    }
}

void EllipsePoint::buildR( double g, double h, double S1, double S2, cv::Matx33d& outMatrix ) 
{
    outMatrix(0,0) = g;
    outMatrix(0,1) = 0.0;
    outMatrix(0,2) = S2*h;

    outMatrix(1,0) = 0.0;
    outMatrix(1,1) = -S1;
    outMatrix(1,2) = 0.0;

    outMatrix(2,0) = S1*S2*h;
    outMatrix(2,1) = 0.0;
    outMatrix(2,2) = -S1*g;
}

bool EllipsePoint::checkN( const cv::Matx33d& VR ) 
{
    // Ellipse normal transformed with VR is equal to VR*(0,0,1)'
    // => equal to the 3rd colomn of VR
    // Check calculate if (VR.col(3))*(0,0,1)' > 0
    return ( VR(2,2) > 0.0 );
}


cv::Point2d EllipsePoint::getCenterOnNormPlane( ) const 
{
    if( !cacheValidCenterNormPlane ) 
    {
        centerNormPlaneCache = ellipseCenter( ellipse_norm );
        cacheValidCenterNormPlane = true;
    }
    return centerNormPlaneCache;
}

cv::Point2d EllipsePoint::getCenter( ) const 
{
    if( !cacheValidCenter ) 
    {
        centerCache = ellipseCenter( ellipse );
        cacheValidCenter = true;
    }
    return centerCache;
}
