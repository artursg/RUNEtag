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
#include "auxmath.hpp"
#include <iomanip>
#include <iostream>

namespace cv{
namespace runetag {

extern double TDelta( const cv::Mat& T1, const cv::Mat& T2 ) {
    cv::Mat Td = T2 - T1;
    return cv::norm( Td );
}

extern double RDelta( const cv::Mat& R1, const cv::Mat& R2 ) {
    cv::Matx31d rv1, rv2;

    if( R1.cols != 3 || R1.rows != 3 ) {
        cv::Mat R1_R;
        cv::Rodrigues( R1, R1_R );
        rv1(0,0) = R1_R.at<double>(0,2);
        rv1(1,0) = R1_R.at<double>(1,2);
        rv1(2,0) = R1_R.at<double>(2,2);
    } else {
        rv1(0,0) = R1.at<double>(0,2);
        rv1(1,0) = R1.at<double>(1,2);
        rv1(2,0) = R1.at<double>(2,2);
    }


    if( R2.cols != 3 || R2.rows != 3 ) {
        cv::Mat R2_R;
        cv::Rodrigues( R2, R2_R );
        rv2(0,0) = R2_R.at<double>(0,2);
        rv2(1,0) = R2_R.at<double>(1,2);
        rv2(2,0) = R2_R.at<double>(2,2);
    } else {
        rv2(0,0) = R2.at<double>(0,2);
        rv2(1,0) = R2.at<double>(1,2);
        rv2(2,0) = R2.at<double>(2,2);
    }

    return acos( rv1(0,0)*rv2(0,0) + rv1(1,0)*rv2(1,0) + rv1(2,0)*rv2(2,0) );
}


cv::RotatedRect ellipseToRotatedRect( const cv::Matx33d& ellipse ) {

    const double& a = ellipse(0,0);
    const double& b = ellipse(0,1);
    const double& c = ellipse(1,1);
    const double& d = ellipse(0,2);
    const double& f = ellipse(1,2);
    const double& g = ellipse(2,2);

    if( b*b - 4.0*a*c >= 0.0 ) {
        return cv::RotatedRect( cv::Point2f(0.0f,0.0f), cv::Size2f(-1.0, -1.0), 0.0f );
    }
    cv::RotatedRect rr;

    const double b2minac = b*b - a*c;
    double x0 = (c*d - b*f) / b2minac;
    double y0 = (a*f - b*d) / b2minac;
    rr.center.x = static_cast<float>(x0);
    rr.center.y = static_cast<float>(y0);

    const double num = 2.0*( a*f*f + c*d*d + g*b*b - 2*b*d*f - a*c*g );
    double a1 = sqrt(  num / (b2minac*( sqrt( (a-c)*(a-c)+4*b*b ) - (a+c) ) )  );
    double b1 = sqrt(  num / (b2minac*( -sqrt( (a-c)*(a-c)+4*b*b ) - (a+c) ) )  );

    if( a1>b1 ) {
        rr.size.width = static_cast<float>(a1);
        rr.size.height = static_cast<float>(b1);
    } else {
        rr.size.width = static_cast<float>(b1);
        rr.size.height = static_cast<float>(a1);
    }

    double theta = 0.0;
    
    if( almost_zero( b, 1e-8) && a>c ) {
        theta = 1.57079632;
    }
    double v = (a-c)/(2*b);
    if( !almost_zero(b, 1e-8 ) && a<c ) {
        theta = 0.5*atan(1.0/v);
    }
    else if( !almost_zero(b, 1e-8 ) && a>c ) {
        theta = 1.57079632 + 0.5*atan(1.0/v);
    }

    //rr.angle = (float)(1.57079632 - theta)*180.0/3.14159265;
    rr.angle = static_cast<float>(theta*180.0/3.14159265);

    rr.angle = static_cast<float>(rr.angle-180.0);
    while( rr.angle >= 360.0 ) {
        rr.angle -= 360.0;
    }

    return rr;
}

cv::Matx33d transformToEllipse( const cv::Matx33d& Qc, const cv::Matx33d& VR, double k ) {

    return (1/k)*VR*Qc*VR.t();
}

cv::Point2d ellipseCenter( const cv::Matx33d& e ) {
    const double& e00 = e(0,0);
    const double& e01 = e(0,1);
    const double& e11 = e(1,1);
    const double& e02 = e(0,2);
    const double& e12 = e(1,2);

    const double b2minac = e01*e01 - e00*e11;

    const double centerx = ( e11*e02 - e01*e12 ) / b2minac;
    const double centery = ( e00*e12 - e01*e02 ) / b2minac;
    return cv::Point2d( centerx, centery );
}


double ellipseRadius( const cv::Matx33d& Qc ) {
    const double a = Qc(0,0);
    const double b = Qc(0,1);
    const double c = Qc(1,1);
    const double d = Qc(0,2);
    const double f = Qc(1,2);
    const double g = Qc(2,2);
    const double num = a*f*f + c*d*d + g*b*b - 2*b*d*f - a*c*g;
    const double denum= (b*b-a*c)*(b*b-a*c)*( (a+c)*(a+c) - ((a-c)*(a-c) + 4*b*b) );
    return sqrt( -2*num*(b*b-a*c)*(a+c)/denum );
}


cv::Matx33d& scalarDivision( cv::Matx33d& targetMatrix, double scalar ) {

    if (!almost_zero(scalar, 10e-18)) {

        targetMatrix(0,0) /= scalar;
        targetMatrix(0,1) /= scalar;
        targetMatrix(0,2) /= scalar;

        targetMatrix(1,0) /= scalar;
        targetMatrix(1,1) /= scalar;
        targetMatrix(1,2) /= scalar;

        targetMatrix(2,0) /= scalar;
        targetMatrix(2,1) /= scalar;
        targetMatrix(2,2) /= scalar;
    }

    return targetMatrix;

}

}
}
