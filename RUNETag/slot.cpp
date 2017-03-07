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
#include "slot.hpp"

using namespace cv::runetag;
////////////////////////////SLOT CLASS METHODS /////////////////////////

Slot::Slot(): _value(false), _discarded(false), payload(0) {}

Slot::Slot(cv::Matx33d _Qmin, cv::Matx33d _Qmax, const cv::Point2d& _c,
        const cv::Point2d& _v1, const cv::Point2d& _v2) : Qmin(_Qmin), 
        Qmax(_Qmax), 
        _value(false), 
        _discarded(false) 
{
    c = cv::Point2f(static_cast<float> (_c.x), static_cast<float> (_c.y));
    v1 = cv::Point2f(static_cast<float> (_v1.x - _c.x), static_cast<float> (_v1.y - _c.y));
    v2 = cv::Point2f(static_cast<float> (_v2.x - _c.x), static_cast<float> (_v2.y - _c.y));
}

bool Slot::checkInside(const cv::Point2d& p) const 
{
    //Angle boundary
    const float vtx = static_cast<float> (p.x - c.x);
    const float vty = static_cast<float> (p.y - c.y);

    const float v1vt_z = v1.x * vty - v1.y * vtx;
    if (v1vt_z > 0.0)
    {
        return false;
    }
    const float v2vt_z = v2.x * vty - v2.y * vtx;
    if (v2vt_z < 0.0)
    {
        return false;
    }

    //Layer boundary
    const double& x = p.x;
    const double& y = p.y;
    const double& A_max = Qmax(0, 0);
    const double& B_max = Qmax(0, 1);
    const double& C_max = Qmax(1, 1);
    const double& D_max = Qmax(0, 2);
    const double& E_max = Qmax(1, 2);
    const double& F_max = Qmax(2, 2);

    if (A_max * x * x + 2.0 * B_max * x * y + C_max * y * y + 2.0 * D_max
            * x + 2.0 * E_max * y + F_max < 0.0) 
    {
        const double& A_min = Qmin(0, 0);
        const double& B_min = Qmin(0, 1);
        const double& C_min = Qmin(1, 1);
        const double& D_min = Qmin(0, 2);
        const double& E_min = Qmin(1, 2);
        const double& F_min = Qmin(2, 2);

        if ( A_min * x * x + 2.0 * B_min * x * y + C_min * y * y + 2.0
                * D_min * x + 2.0 * E_min * y + F_min > 0.0 ) 
        {
            return true;
        }
    }

    return false;
}

bool Slot::setIfInside(const cv::Point2d& p, EllipsePoint* _payload)
{
    if ( checkInside(p) && _payload->isinside(slot_center.x,slot_center.y))
    {
        _value = true;
        payload = _payload;
        return true;
    }
    return false;
}
