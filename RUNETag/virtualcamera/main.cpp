#include <iostream>
#include <fstream>

#include <cv.h>
#include <highgui.h>
#include "../include/ImgTextStream.hpp"
#include "VirtualCamera.h"

#define IMG_FILENAME "circle.bmp"

#include <time.h>

#define KEY_UP 2490368
#define KEY_DOWN 2621440
#define KEY_LEFT 2424832
#define KEY_RIGHT 2555904
#define KEY_PAGDOWN 2228224
#define KEY_PAGUP 2162688
#define KEY_W   119
#define KEY_S   115
#define KEY_A   97
#define KEY_D   100


int run_interactive_mode() {
    
    std::cout << "Loading source image" << std::endl;

    IplImage* src_img = cvLoadImage( IMG_FILENAME );
	if( src_img == NULL ) {
		std::cout << "Unable to load image " << std::endl;
		return -1;
	}

    cvlab::VirtualCamera* vcam = new cvlab::VirtualCamera( cv::Mat( src_img ) );

    bool running=true;
    bool overlay=true;
    while( running ) {
        
        //cv::Mat dst;
        std::cout << "Rendering...";
        vcam->snap();
        
        /*
        if( overlay ) {
            cvExt::its( dst ) << "T: " << T.at<double>(0,0) << " ; " << T.at<double>(1,0) << " ; " << T.at<double>(2,0) << cvExt::newline
             << "angle x: " << anglex << " angle z: " << anglez << cvExt::newline
             << "f: " << f << " cx: " << cx << " cy: " << cy << cvExt::newline;
        }*/

        IplImage dst_img = (IplImage)vcam->getSnapshot();
        cvShowImage( "image", &dst_img );
        std::cout << "done" << std::endl;

        int code = cvWaitKey(0);
        if( (code & 255) == 27 ) {
            running = false;
            break;
        }
        
        switch( code ) {
            case KEY_UP:
                vcam->translate(0.0, 10.0, 0.0 );
                break;
            case KEY_DOWN:
                vcam->translate(0.0, -10.0, 0.0 );
                break;
            case KEY_LEFT:
                vcam->translate(10.0, 0.0, 0.0 );
                break;
            case KEY_RIGHT:
                vcam->translate(-10.0, 0.0, 0.0 );
                break;
            case KEY_PAGDOWN:
                vcam->translate(0.0, 0.0, -10.0 );
                break;
            case KEY_PAGUP:
                vcam->translate(0.0, 0.0, 10.0 );
                break;
            case KEY_W:
                vcam->rotateOnXAxis( 0.1f );
                break;
            case KEY_S:
                vcam->rotateOnXAxis( -0.1f );
                break;
            case KEY_A:
                vcam->rotateOnZAxis( 0.1f );
                break;
            case KEY_D:
                vcam->rotateOnZAxis( -0.1f );
                break;
            default:
                std::cout << "Unknown code: " << code << std::endl;
                break;
        }
        
        
    }
	
	
	// Release the capture device housekeeping
	cvDestroyWindow( "image" );
	return 0;
}

int main(int argc, char* argv[])
{
    return run_interactive_mode();
}
