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
#include "ellipsedetector.hpp"

using namespace cv::runetag;

unsigned int EllipseDetector::detectEllipses( const cv::Mat& frame, std::vector< cv::RotatedRect >& detected ) 
{
    
    unsigned int num_detected = 0;

    // Conversion of the RGB image to a black-white image
    // requires converting it to grayscale and thresholding it:

    // Conversion of the RGB image to GRAYSCALE
    cv::Mat grayframe( frame.rows, frame.cols, CV_8UC1 );
    cv::cvtColor( frame, grayframe, CV_RGB2GRAY );

    //cv::adaptiveThreshold( grayframe, aux, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 127 , 30);
    //cv::threshold( grayframe, aux, 100 , 255, cv::THRESH_BINARY );
    // Threshold
    cv::Mat thresholded;
    cv::adaptiveThreshold( grayframe, thresholded, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 127 , 15);

    // Detect image contours as a vector of Point
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > h;
    cv::findContours( thresholded, contours, h, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    for( std::vector< std::vector< cv::Point > >::const_iterator it = contours.begin(); it != contours.end(); ++it ) 
    {
        const std::vector< cv::Point > &cont = *it;
        if( cont.size() < min_ellipse_contour_points || cont.size() > max_ellipse_contour_points ) 
        {
            // If contour is greater or smaller than the specified soils
            // it's ignored
            continue;
        }

        // Fitting an Ellipse to the contour
        cv::RotatedRect ellipse = cv::fitEllipse( cv::Mat(cont) );
        
        // Ellipse area check
        double area = ellipse.size.width*ellipse.size.height*3.141592;
        if( area < min_ellipse_area || area > max_ellipse_area )
        {
            // If the area is greater or smaller than the specified soils
            // it's ignored
            continue;
        }

        // Ellipse roundness check
        double ratio = ellipse.size.width / ellipse.size.height;
        if( ratio < min_required_roundness || ratio > 1.0/min_required_roundness )
        {
            // If the roundness is greater or smaller than the specified soils
            // it's ignored
            continue;
        }

        // Checking Min Square Error (MSE)
        double wbox = ellipse.size.width;
        double hbox = ellipse.size.height;
        double angle= ellipse.angle*3.14159265/180.;
        
        if( wbox < hbox )
        {
            wbox = ellipse.size.height;
            hbox = ellipse.size.width;
            angle+=3.14159265/2.;
        }

        double f1_x, f1_y,f2_x,f2_y;
        double focus_len = sqrt(wbox*wbox/4.-hbox*hbox/4.);
        double fx = std::cos(angle)*focus_len;
        double fy = std::sin(angle)*focus_len;
        double center_x = ellipse.center.x;
        double center_y = ellipse.center.y;
        f1_x = center_x - fx;
        f1_y = center_y - fy;
        f2_x = center_x + fx;
        f2_y = center_y + fy;
        double dist_sum_energy = wbox;
        double mse=0.;
        double dk=0., dk1, dk2;
        int count = 0;
        for( unsigned int i=0; i<cont.size(); i++ ) 
        {
            const cv::Point& p = cont[i];
            double contour_pt_x = p.x;
            double contour_pt_y = p.y;

            dk1 = std::sqrt((f1_x-contour_pt_x)*(f1_x-contour_pt_x) + (f1_y-contour_pt_y)*(f1_y-contour_pt_y));
            dk2 = std::sqrt((f2_x-contour_pt_x)*(f2_x-contour_pt_x) + (f2_y-contour_pt_y)*(f2_y-contour_pt_y));

            dk = dk1 + dk2 - dist_sum_energy;
            mse += dk*dk;
            ++count;
        }
        mse = std::sqrt(mse)/(double)count;
        if( mse > max_mse )
        {
            continue;
        }

        ellipse.angle = static_cast<float>(180.0-ellipse.angle);

        //Normalize angle
        //if( ellipse.angle < 0.0 )
        //	ellipse.angle = 360.0-ellipse.angle;
        
        while( ellipse.angle >= 360.0 ) 
        {
            ellipse.angle -= 360.0;
        }

        
        // This partially compensates the fact that adaptiveThreshold overestimate the size of the ellipses.
        ellipse.size.width += size_compensation;
        ellipse.size.height += size_compensation;


        detected.push_back( ellipse );
        ++num_detected;
        //cv::ellipse( cv::Mat(frame), ellipse.center, cv::Size( ellipse.size.width/2.0, ellipse.size.height/2.0 ), ellipse.angle, 0, 360, CV_RGB(255,0,0), 1 );

        /*
        for( unsigned int k=0; k<contour_points.size(); k++ ) {
            cv::circle( cv::Mat(frame), contour_points[k], 1, CV_RGB(0,255,0), 1);
        }*/
    }

    return num_detected;
}
