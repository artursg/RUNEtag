/**
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

#include "test_occlusion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <time.h>

using cv::runetag::EllipsePoint;

namespace RUNEtagdetect
{
	namespace test
	{
		namespace occlusions
		{
			void print_code( std::ostream& os, std::vector<long> code )
			{
				for( size_t i=0; i<code.size(); ++i )
					os << code[i] << " ";
				os << std::endl;
			}


			void print_code( std::ostream& os, std::vector<bool> code )
			{
				for( size_t i=0; i<code.size(); ++i )
					os << (code[i]?"1":"0") << " ";
				os << std::endl;
			}
		}
	}
}



void RUNEtagdetect::test::occlusions::test_occlusions( const cv::runetag::MarkerDetected& tag, 
													   unsigned int ntests, 
													   bool interactive, 
													   std::string out_file,
													   cv::Mat input_img )
{
	srand( time(0));
	cv::runetag::Coding coding;
    coding.init();

	// Compute tag bounding box
	unsigned int bbox_xmin = input_img.cols;
	unsigned int bbox_xmax = 0;
	unsigned int bbox_ymin = input_img.rows;
	unsigned int bbox_ymax = 0;
	for( unsigned int idx=0; idx<tag.getNumSlots(); ++idx )
	{
		if( tag.getSlot(idx).value() )
		{
			cv::RotatedRect rr = tag.getSlot(idx).getPayload()->get_as_rotated_rect();
			cv::Rect brect = rr.boundingRect();
			if( brect.x < bbox_xmin )
				bbox_xmin = brect.x;
			if( brect.x+brect.width > bbox_xmax )
				bbox_xmax = brect.x+brect.width;
			if( brect.y < bbox_ymin )
				bbox_ymin = brect.y;
			if( brect.y+brect.height > bbox_ymax )
				bbox_ymax = brect.y+brect.height;
		}
	}
	cv::Rect bbox_rect( cv::Point(bbox_xmin,bbox_ymin), cv::Point(bbox_xmax,bbox_ymax));


	std::ofstream ofs_data( out_file.c_str() );
	ofs_data << "TAG_BOX_AREA" << " " << "OCCLUDED_AREA" << " " << "OCCLUSION_PERCENT" << " " << "NUM_ERASURES" << " " << "DETECTION_RESULT(0/1)" << std::endl;

	for( int test_i=0; test_i<ntests; ++test_i )
	{
		// Randomize occlusion box
		cv::Point pt1( rand()%(bbox_xmax-bbox_xmin) + bbox_xmin, rand()%(bbox_ymax-bbox_ymin) + bbox_ymin );
		cv::Point pt2( rand()%(bbox_xmax-bbox_xmin) + bbox_xmin, rand()%(bbox_ymax-bbox_ymin) + bbox_ymin );
		cv::Rect occlusionbox( pt1, pt2 );
		

		std::vector< bool > bcode( tag.getNumSlots(), false );
		std::vector< cv::Point > point_added;
		std::vector< cv::Point > point_occluded;
		for( int idx=0; idx<tag.getNumSlots(); ++idx )
		{
			if( tag.getSlot(idx).value() )
			{
				cv::RotatedRect rr = tag.getSlot(idx).getPayload()->get_as_rotated_rect();
				if( !occlusionbox.contains( rr.center ) )
				{
					bcode[idx] = tag.getSlot(idx).value();
					point_added.push_back( rr.center );
				} else 
				{
					point_occluded.push_back( rr.center );
				}
			}			
		}


		std::vector< long > detected_code = coding.pack( bcode );
        //std::cout << "Binary code: " << std::endl;
        //print_code(std::cout, bcode);
        ///std::cout << "Code: " << std::endl;
		//RUNEtagdetect::test::occlusions::print_code(std::cout, detected_code);

		bool detection_ok=false;
		try {
			if( coding.decode( detected_code ) == 0 )
			{
				// Decoding was successful
				//std::cout << "Decoded: " << std::endl;
				//RUNEtagdetect::test::occlusions::print_code(std::cout, detected_code);
				long idx_detected;
				long rotation;
				std::vector<long> aligned_code;
				coding.align( detected_code, idx_detected, rotation );

				if( idx_detected == tag.associatedModel()->getIDX() )
				{
					detection_ok=true;
				}
			}
		} catch( std::invalid_argument& ex )
		{
			// Do nothing
		}

		

		ofs_data << bbox_rect.area() << " " << occlusionbox.area() << " " << (((double)occlusionbox.area()/(double)bbox_rect.area())*100.0) << " " << point_occluded.size() << " " << (detection_ok?"1":"0") << std::endl;
		

		if( interactive )
		{
			cv::Mat dbgimg = input_img.clone();
			
			// Draw bbox 
			cv::rectangle( dbgimg, cv::Point(bbox_xmin,bbox_ymin), cv::Point(bbox_xmax,bbox_ymax),CV_RGB(255,0,0),1,CV_AA);

			// Draw occlusion box
			cv::rectangle( dbgimg, occlusionbox,CV_RGB(255,255,0),CV_FILLED,CV_AA);

			for( std::vector< cv::Point >::const_iterator it=point_added.begin(); it!=point_added.end(); ++it )
			{
				cv::circle( dbgimg, *it, 5, CV_RGB(0,255,0),CV_FILLED, CV_AA );
			}
			for( std::vector< cv::Point >::const_iterator it=point_occluded.begin(); it!=point_occluded.end(); ++it )
			{
				cv::circle( dbgimg, *it, 5, CV_RGB(255,0,0),CV_FILLED, CV_AA );
			}

			cv::Mat dbgimg_scaled;
			cv::resize(dbgimg,dbgimg_scaled,cv::Size(640,480) );
			cv::imshow("Occlusion test",dbgimg_scaled );
			cv::waitKey(10);
		}
	}
	ofs_data.flush();
	ofs_data.close();

	return;
}
