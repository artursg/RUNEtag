#ifndef _RUNETAG_DETECT_TEST_OCCLUSIONS_H_
#define _RUNETAG_DETECT_TEST_OCCLUSIONS_H_

#include "runetag.hpp"

namespace RUNEtagdetect
{
	namespace test
	{
		namespace occlusions
		{
			extern void test_occlusions( const cv::runetag::MarkerDetected& tag, 
				                         unsigned int ntests, 
										 bool interactive, 
										 std::string out_file,
										 cv::Mat input_img );
		}


	}
}


#endif