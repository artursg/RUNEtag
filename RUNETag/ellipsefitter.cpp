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
#include "ellipsefitter.hpp"

//#include "auxmath.hpp"
#include <limits>

using std::vector;
using namespace cv::runetag;

/***********************************
 * EllipseFitter Aux structure
 ***********************************/
EllipseFitter::CandidateVR::CandidateVR( const cv::Matx33d& _VR1, const cv::Matx31d& _normal1, const cv::Matx33d& _VR2, const cv::Matx31d& _normal2 ) : VR1(&_VR1), normal1(&_normal1), VR2(&_VR2), normal2(&_normal2) {}

double EllipseFitter::CandidateVR::cosAngle() const 
{
    const cv::Matx31d& v1 = *normal1;
    const cv::Matx31d& v2 = *normal2;
    double dotProduct = v1(0,0)*v2(0,0) + v1(1,0)*v2(1,0) + v1(2,0)*v2(2,0);
    return dotProduct;
}


bool EllipseFitter::chooseAvgVR( const EllipsePoint& ep1, const EllipsePoint& ep2, double& r, double& r_other, cv::Matx33d& Qc, cv::Matx33d& Qc_other ) 
{
    // Get 4 normals (for each two ellipse point, for each VR)
    cv::Matx31d Q1_VR1_normal( buildN( ep1.getVR1() ));
    cv::Matx31d Q1_VR2_normal( buildN( ep1.getVR2() ));
    cv::Matx31d Q2_VR1_normal( buildN( ep2.getVR1() ));
    cv::Matx31d Q2_VR2_normal( buildN( ep2.getVR2() ));

    //Possible VR to be tested are:  
    // (Q1_VR1,Q2_VR1) (Q1_VR1,Q2_VR2) (Q1_VR2,Q2_VR1) (Q1_VR2,Q2_VR2)
    CandidateVR q1vr1_q2vr1( ep1.getVR1(), Q1_VR1_normal, ep2.getVR1(), Q2_VR1_normal );
    CandidateVR q1vr1_q2vr2( ep1.getVR1(), Q1_VR1_normal, ep2.getVR2(), Q2_VR2_normal );
    CandidateVR q1vr2_q2vr1( ep1.getVR2(), Q1_VR2_normal, ep2.getVR1(), Q2_VR1_normal );
    CandidateVR q1vr2_q2vr2( ep1.getVR2(), Q1_VR2_normal, ep2.getVR2(), Q2_VR2_normal );

    vector<CandidateVR> candidates;
    //Normal filtering
    if( q1vr1_q2vr1.cosAngle() > coplanar_threshold ) 
    {
        candidates.push_back( q1vr1_q2vr1 );
    }
    if( q1vr1_q2vr2.cosAngle() > coplanar_threshold ) 
    {
        candidates.push_back( q1vr1_q2vr2 );
    }
    if( q1vr2_q2vr1.cosAngle() > coplanar_threshold ) 
    {
        candidates.push_back( q1vr2_q2vr1 );
    }
    if( q1vr2_q2vr2.cosAngle() > coplanar_threshold ) 
    {
        candidates.push_back( q1vr2_q2vr2 );
    }
    
    if( candidates.size() == 0 )
    {
        //ep1 ep2 are not co-planar
        return false;
    }

    double min_err = std::numeric_limits< double >::max();
    const cv::Matx33d& Q1 = ep1.getEllipseNorm();
    const cv::Matx33d& Q2 = ep2.getEllipseNorm();

    for( vector<CandidateVR>::iterator it= candidates.begin(); it!=candidates.end(); it++ ) 
    {
        //For each candidate we create the average rotation
        Quaternion<double> qr1( it->getVR1() );
        Quaternion<double> qr2( it->getVR2() );

        Quaternion<double> qavg = qr1.dot( qr2 ) > 0.0 ? qr1 + qr2 : qr1-qr2;
        qavg.normalize();
        cv::Matx33d VR_avg = qavg.toRotationMatrix();

        cv::Matx33d Q1c_vravg, Q2c_vravg;
        transformToCircle( Q1, VR_avg, Q1c_vravg );
        transformToCircle( Q2, VR_avg, Q2c_vravg );
        const double rq1 = circleRonZ0( Q1c_vravg );
        const double rq2 = circleRonZ0( Q2c_vravg );
        const double err = fabs( rq1-rq2 ) / (rq1+rq2);

        if( err < min_err ) 
        {
            VR = VR_avg;
            r = rq1;
            r_other = rq2;
            min_err = err;
            Qc = Q1c_vravg;
            Qc_other = Q2c_vravg;
        }
    }

    if( min_err > size_err_threshold ) 
    {
        return false;
    }

    return true;
}

bool EllipseFitter::fitEllipseAvg( double radius_ratio ) 
{
    double r1;
    double r2;
    cv::Matx33d Qc1;
    cv::Matx33d Qc2;

    if( !chooseAvgVR( e1, e2, r1, r2, Qc1, Qc2) )
    {
        return false;
    }
    
    const cv::Point2d center1 = ellipseCenter( Qc1 );
    const cv::Point2d center2 = ellipseCenter( Qc2 );
    
    const double rad_avgSqr = (r1 + r2)*0.5;
    const double radiusSqr = rad_avgSqr*radius_ratio*radius_ratio;
    const double dsqr = (center1.x-center2.x)*(center1.x-center2.x) + 
                        (center1.y-center2.y)*(center1.y-center2.y);

    /*
    if( dsqr > radiusSqr*2.0  || dsqr < radiusSqr*0.4  )
    {
        return false;
    }
    */

    rad_avg = sqrt( rad_avgSqr );
    radius = sqrt( radiusSqr );

    bool canfit = fitCircles( center1, center2, radiusSqr, Qfit1c, Qfit2c );
    if( !canfit )
    {
        return false;
    }
    
    return true;

}


// pass another parameter to use transformToEllipse with a specific 3° parameter
cv::Matx33d EllipseFitter::getFitWithOffset( double rad_mul, cv::Matx33d& who, double k) const 
{
    const double f = ( intrinsics.at<double>(0,0) + intrinsics.at<double>(1,1)) / 2.0;
    const double aux = who(2,2);
    double rz0sqr = ( radius + rad_avg*rad_mul )*( radius + rad_avg*rad_mul );

    who(2,2) = -rz0sqr + who(0,2)*who(0,2) + who(1,2)*who(1,2);
    
    cv::Matx33d fit_off = transformToEllipse( who, VR, k);
    fit_off(0,2) *= f;
    fit_off(1,2) *= f;
    fit_off(2,0) *= f;
    fit_off(2,1) *= f;
    fit_off(2,2) *= f*f;

    who(2,2) = aux;

    return fit_off;
} 

cv::Matx33d EllipseFitter::getFit1WithOffset( double rad_mul ) const 
{
    return getFitWithOffset(rad_mul, Qfit1c, e1.getL2inv());
}


cv::Matx33d EllipseFitter::getFit2WithOffset( double rad_mul ) const 
{
    //TODO: verify e2.getL2inv()
    return getFitWithOffset(rad_mul, Qfit2c, e2.getL2inv());
}


cv::Matx33d& EllipseFitter::transformToCircle( const cv::Matx33d& Q, const cv::Matx33d& VR, cv::Matx33d& targetMatrix ) 
{
        targetMatrix = VR.t()*Q*VR;
        double nr = (targetMatrix(0,0) + targetMatrix(1,1)) / 2.0;
        return scalarDivision(targetMatrix, nr);
}


cv::Matx31d EllipseFitter::buildN( const cv::Matx33d& VR ) 
{
    //Ellipse normal transformed with VR is equal to VR*(0,0,1)'
    //that is equal to the column 2 of VR
    cv::Matx31d n;
    n(0,0) = VR(0,2);
    n(1,0) = VR(1,2);
    n(2,0) = VR(2,2);
    normalize(n);
    return n;
}



double EllipseFitter::circleRonZ0( const cv::Matx33d& Qc ) 
{
    double a = Qc(0,0);
    double b = Qc(0,1);
    double c = Qc(1,1);
    double d = Qc(0,2);
    double f = Qc(1,2);
    double g = Qc(2,2);
    double num = a*f*f + c*d*d + g*b*b - 2*b*d*f - a*c*g;
    //double denum= (b*b-a*c)*(b*b-a*c)*( (a+c)*(a+c) - ((a-c)*(a-c) + 4*b*b) );
    //return -2*num*(b*b-a*c)*(a+c)/denum;
    double denum= (b*b-a*c)*( (a+c)*(a+c) - ((a-c)*(a-c) + 4*b*b) );
    return -2*num*(a+c)/denum;
}

bool EllipseFitter::fitCircles( const cv::Point2d& center1, const cv::Point2d& center2, double radius2, cv::Matx33d& circle1, cv::Matx33d& circle2 ) 
{
    const double x1=center1.x; 
    const double y1=center1.y;
    const double x2=center2.x;
    const double y2=center2.y;

    const double asqrt = (  -(x1 - x2)*(x1 - x2) * ( x1*x1 - 2*x1*x2 +  x2*x2 + (y1 - y2)*(y1 - y2) ) * 
                         (-4*radius2 + x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2) );
    if( asqrt < 0.0 )
    {
        return false;
    }

    const double bsqrt = (-(x1 - x2)*(x1 - x2) * (x1*x1 - 2*x1*x2 +  x2*x2 + (y1 - y2)*(y1 - y2) ) * 
                         (-4*radius2 + x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2));
    if( bsqrt<0.0 )
    {
        return false;
    }

    const double anum = (-x1*x1*x1*x1 + 2*x1*x1*x1*x2 - 2*x1*x2*x2*x2 + x2*x2*x2*x2 - x1*x1*(y1 - y2)*
                        (y1 - y2) + x2*x2 * (y1 - y2)*(y1 - y2) );
    const double adenum = (2*(x1 - x2)*(x1*x1 - 2*x1*x2 +  x2*x2 + (y1 - y2)*(y1 - y2) ));

    const double bnum = -(y1*y1*y1 - y1*y1*y2 - y1*y2*y2 + y2*y2*y2 + x1*x1*(y1 + y2) - 
                         2*x1*x2*(y1 + y2) +  x2*x2*(y1 +  y2) );
    const double bdenum = (2*(x1*x1 - 2*x1*x2 +   x2*x2 + (y1 - y2)*(y1 - y2)));

    const double a2 = (anum + (-y1+y2) * sqrt(asqrt)) / adenum;
    const double a1 = (anum + ( y1-y2) * sqrt(asqrt)) / adenum;

    const double b1 = (bnum - sqrt(bsqrt)) / bdenum;
    const double b2 = (bnum + sqrt(bsqrt)) / bdenum;

    //Build circle1
    buildCircle( circle1, a1, b1, radius2 );

    //Build circle2
    buildCircle( circle2, a2, b2, radius2);

    return true;
}

void EllipseFitter::buildCircle( cv::Matx33d& m, double a, double b, double radius ) 
{	
    m = cv::Matx33d();

    m(0,0) = 1.0;
    m(0,1) = 0.0;
    m(0,2) = a;

    m(1,0) = 0.0;
    m(1,1) = 1.0;
    m(1,2) = b;

    m(2,0) = a;
    m(2,1) = b;
    m(2,2) = a*a + b*b - radius;
}

void EllipseFitter::normalize( cv::Matx31d& vector ) 
{
    double magnitude = sqrt( vector(0,0)*vector(0,0) + vector(1,0)*vector(1,0) + vector(2,0)*vector(2,0) );
    vector(0,0) /= magnitude;
    vector(1,0) /= magnitude;
    vector(2,0) /= magnitude;
}