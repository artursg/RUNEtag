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

#ifndef ELLIPSE_FITTER_H
#define ELLIPSE_FITTER_H

//#include "precomp.h"

#include "ellipsepoint.hpp"
#include "auxmath.hpp"

namespace cv
{
namespace runetag
{

class EllipseFitter {

public:
    cv::Matx33d VR;

    private:

    const double coplanar_threshold;
    const double size_err_threshold;

    const EllipsePoint& e1;
    const EllipsePoint& e2;
    const cv::Mat& intrinsics;

    mutable cv::Matx33d Qfit1c;
    mutable cv::Matx33d Qfit2c;

    double rad_avg;
    double radius;

    /*
    Main operation provided by getFit1WithOffset and getFit2WithOffset
    */
    cv::Matx33d getFitWithOffset( double rad_mul, cv::Matx33d& Qfit, double k ) const;

    bool chooseAvgVR( const EllipsePoint& ep1, const EllipsePoint& ep2, double& r, double& r_other, cv::Matx33d& Qc, cv::Matx33d& Qc_other );

    // AUX OPERATION
    static cv::Matx33d& transformToCircle( const cv::Matx33d& Q, const cv::Matx33d& VR, cv::Matx33d& targetMatrix );
    static void normalize( cv::Matx31d& vector );
    static cv::Matx31d buildN( const cv::Matx33d& VR );
    static double circleRonZ0( const cv::Matx33d& Qc );
    static bool fitCircles( const cv::Point2d& center1, const cv::Point2d& center2, double radius2, cv::Matx33d& circle1, cv::Matx33d& circle2 );
    static void buildCircle( cv::Matx33d& target, double a, double b, double radius );

    // AUX STRUCTURE
    class CandidateVR 
    {

    private:
        const cv::Matx33d* VR1;
        const cv::Matx33d* VR2;
        const cv::Matx31d* normal1;
        const cv::Matx31d* normal2;

    public:

        CandidateVR( const cv::Matx33d& _VR1, const cv::Matx31d& _normal1, const cv::Matx33d& _VR2, const cv::Matx31d& _normal2 );

        inline const cv::Matx33d& getVR1() const 
        { 
            return *VR1;
        }
        
        inline const cv::Matx33d& getVR2() const
        {
            return *VR2;
        }
        
        inline const cv::Matx31d& getNormal1() const
        {
            return *normal1;
        }

        inline const cv::Matx31d& getNormal2() const
        {
            return *normal2;
        }

        double cosAngle() const;
    };

    template <typename T>
    class Quaternion
    {
    private:
        T w, i, j, k;

    public:
        explicit Quaternion() : w(0), i(0), j(0), k(0) {}
        explicit Quaternion(const T* p) : w(p[0]), i(p[1]), j(p[2]), k(p[3]) {}
        explicit Quaternion( const cv::Matx33d& R );
        Quaternion(T ww, T ii, T jj, T kk) : w(ww), i(ii), j(jj), k(kk) {}


        inline void normalize();

        cv::Matx33f toRotationMatrix() const;

        Quaternion<T> operator+( const Quaternion<T>& q ) const;
        Quaternion<T> operator-( const Quaternion<T>& q ) const;
        Quaternion<T> operator*( const Quaternion<T>& q ) const;

        T dot( const Quaternion<T>& q ) const;
    };

public:

    EllipseFitter( const EllipsePoint& _e1, const EllipsePoint& _e2, const cv::Mat& _intrinsics, double _coplanar_threshold=0.97, double _size_err_threshold=0.5 ) : e1( _e1 ), e2( _e2 ), intrinsics(_intrinsics), coplanar_threshold(_coplanar_threshold), size_err_threshold(_size_err_threshold) {}

    inline const cv::Matx33d& getVR() const 
    {
        return VR;
    }

    // MAIN OPERATIONS
    bool fitEllipseAvg( double _radius_ratio );
    cv::Matx33d getFit1WithOffset( double rad_mul ) const;
    cv::Matx33d getFit2WithOffset( double rad_mul ) const;
    
};

    /*
     * BEGIN TEMPLATE CLASS IMPLEMENTATION
     */
    template<typename T>
    EllipseFitter::Quaternion<T>::Quaternion( const cv::Matx33d& m )
    {
        const T m00 = m(0,0);
        const T m11 = m(1,1);
        const T m22 = -m(2,2);
        const T m01 = m(0,1);
        const T m02 = -m(0,2);
        const T m10 = m(1,0);
        const T m12 = -m(1,2);
        const T m20 = m(2,0);
        const T m21 = m(2,1);
        const T tr = m00 + m11 + m22;

        if (tr > 0)
        {
            T s = std::sqrt(tr+T(1)) * 2; // S=4*qw
            w = 0.25 * s;
            i = (m21 - m12) / s;
            j = (m02 - m20) / s;
            k = (m10 - m01) / s;
        }
        else if ((m00 > m11)&&(m00 > m22))
        {
            const T s = std::sqrt(T(1) + m00 - m11 - m22) * 2; // S=4*qx
            w = (m21 - m12) / s;
            i = 0.25 * s;
            j = (m01 + m10) / s;
            k = (m02 + m20) / s;
        }
        else if (m11 > m22)
        {
            const T s = std::sqrt(T(1) + m11 - m00 - m22) * 2; // S=4*qy
            w = (m02 - m20) / s;
            i = (m01 + m10) / s;
            j = 0.25 * s;
            k = (m12 + m21) / s;
        }
        else
        {
            const T s = std::sqrt(T(1) + m22 - m00 - m11) * 2; // S=4*qz
            w = (m10 - m01) / s;
            i = (m02 + m20) / s;
            j = (m12 + m21) / s;
            k = 0.25 * s;
        }
    }

    template<typename T>
    EllipseFitter::Quaternion<T> EllipseFitter::Quaternion<T>::operator -( const Quaternion<T>& q ) const
    {
        return Quaternion<T> (w - q.w, i - q.i, j - q.j, k - q.k);
    }

    template<typename T>
    EllipseFitter::Quaternion<T> EllipseFitter::Quaternion<T>::operator +( const Quaternion<T>& q ) const 
    {
        return Quaternion<T> (w + q.w, i + q.i, j + q.j, k + q.k);
    }

    template<typename T>
    EllipseFitter::Quaternion<T> EllipseFitter::Quaternion<T>::operator *( const Quaternion<T>& q ) const
    {
        return Quaternion<T>(w*q.w - (i*q.i + j*q.j + k*q.k),
                             w*q.i + q.w*i + j*q.k - k*q.j,
                             w*q.j + q.w*j + k*q.i - i*q.k,
                             w*q.k + q.w*k + i*q.j - j*q.i);
    }

    template<typename T>
    T EllipseFitter::Quaternion<T>::dot( const Quaternion<T>& q ) const
    {
        return w*q.w + i*q.i + j*q.j + k*q.k;
    }

    template<typename T>
    cv::Matx33f EllipseFitter::Quaternion<T>::toRotationMatrix() const
    {
        cv::Matx33f matrix;

        matrix(0,0) = static_cast<float>(1 - 2*j*j - 2*k*k);
        matrix(0,1) = static_cast<float>(2*i*j - 2*k*w);
        matrix(0,2) = static_cast<float>(-(2*i*k + 2*j*w));

        matrix(1,0) = static_cast<float>(2*i*j + 2*k*w);
        matrix(1,1) = static_cast<float>(1 - 2*i*i - 2*k*k);
        matrix(1,2) = static_cast<float>(-(2*j*k - 2*i*w));

        matrix(2,0) = static_cast<float>(2*i*k - 2*j*w);
        matrix(2,1) = static_cast<float>(2*j*k + 2*i*w);
        matrix(2,2) = static_cast<float>(-(1 - 2*i*i - 2*j*j));

        return matrix;
    }

    template<typename T>
    inline void EllipseFitter::Quaternion<T>::normalize()
    {
        T magnitude = w * w + i * i + j * j + k * k;
        if (!almost_zero(magnitude - T(1), 1e-10))
        {
            magnitude = std::sqrt(magnitude);
            w /= magnitude;
            i /= magnitude;
            j /= magnitude;
            k /= magnitude;
        }
    }

    /*
     * END TEMPLATE CLASS IMPLEMENTATION
     */

} // namespace runetag
} // namespace cv

#endif
