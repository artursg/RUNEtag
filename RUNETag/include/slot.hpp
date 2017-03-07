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


#ifndef _SLOT_H
#define _SLOT_H

//#include "precomp.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ellipsepoint.hpp"

namespace cv 
{
namespace runetag
{

class Slot 
{

    friend class SlotFitter;

private:

    
    cv::Matx33d Qmin;
    cv::Matx33d Qmax;
    bool _value;
    bool _discarded;
    EllipsePoint* payload;

public:

    cv::Point2f v1;
    cv::Point2f v2;
    cv::Point2f c;
    cv::Point2f slot_center;

    Slot();

    Slot(cv::Matx33d _Qmin, cv::Matx33d _Qmax, const cv::Point2d& _c, const cv::Point2d& _v1, const cv::Point2d& _v2);

    inline void invalidate() 
    {
        _value = false;
        _discarded = true; /*payload = 0;*/
    }

    inline bool value() const 
    {
        return _value;
    }

    inline bool discarded() const 
    {
        return _discarded;
    }

    inline EllipsePoint* getPayload() const 
    {
        return payload;
    }

    inline cv::Point2d getCenter() const
    {
        return c;
    }

    bool checkInside(const cv::Point2d& p) const;

    bool setIfInside(const cv::Point2d& p, EllipsePoint* _payload);
};

} // namespace runetag
} // namespace cv

#endif
