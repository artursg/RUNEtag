#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include "runetag.hpp"
#include "auxrenderer.hpp"
#include "VirtualCamera.h"
#include "ellipsefitter.hpp"

#include "markerpose.hpp"
#include "imgtextstream.hpp"

#include "coding.h"

using namespace cv;



std::vector< cv::Vec2d > extract_points_location( cv::Mat frame, cv::Mat intrinsics, std::vector< RotatedRect > ellipses )
{
    std::vector< cv::Vec2d > loc;

    cv::runetag::MarkerDetector markerDetector( intrinsics );
    markerDetector.addModelsFromFile( "TestMarker.txt" );
    
    std::vector< cv::runetag::MarkerDetected > tags_found;
    markerDetector.detectMarkers( ellipses, tags_found);

    if( tags_found.size()>0 )
    {
        const cv::runetag::MarkerDetected& tag = tags_found[0];
        /* */
        //debug
        cv::Mat dbg = frame.clone();
        for( unsigned int i=0; i<tag.getNumSlots(); i++ ) 
        {
            if( tag.getSlot(i).value() ) {
                cv::Point2f pos( static_cast<float>( tag.getSlot(i).getCenter().x + intrinsics.at<double>(0,2)), 
                    static_cast<float>( tag.getSlot(i).getCenter().y + intrinsics.at<double>(1,2)) );

                cv::runetag::its( dbg ) << cv::runetag::its::position( pos ) << i;
            }
        }
        cv::imshow( "debug", dbg );
        cv::waitKey(0);
        /**/
    }
    
    return loc;
}




//#define CREATE_SNAPSHOT
#if 1

int main (int argc, char** argv)
{

    cv::Mat frame;

    double focal_len=1000;
    double angle=45;
    
    cv::Mat intrinsics = cv::Mat::eye(3,3,CV_64FC1);


#ifdef CREATE_SNAPSHOT
    {
        std::string img_file("virtualcamera/testdirect.png");

        cv::Mat vcimg = cv::imread( img_file );
        if(vcimg.rows==0 || vcimg.cols==0 )
        {
            std::cout << "Unable to load " << img_file << std::endl;
            return 0;
        }

        std::cout << "Virtual camera image loaded" << std::endl;

        std::cout << "Creating snapshot" << std::endl;
        cvlab::VirtualCamera vc(vcimg, focal_len, focal_len);
        vc.translate(0,0,0);
        vc.rotateOnXAxis( -angle*0.017453 );
        vc.snap();

        frame = vc.getSnapshot();
        intrinsics = vc.getIntrinsics();
        cv::imwrite("frame.png", frame );
    }
#else
    frame = cv::imread("frame.png");
    
    intrinsics.at<double>(0,0) = focal_len;
    intrinsics.at<double>(1,1) = focal_len;
    intrinsics.at<double>(0,2) = frame.cols/2.0;
    intrinsics.at<double>(1,2) = frame.rows/2.0;
#endif


    cv::rectangle( frame, cv::Point2i(600,0), cv::Point2i( 1000,600), CV_RGB(255,255,255), -1, CV_AA );

    double cx = intrinsics.at<double>(0,2);
    double cy = intrinsics.at<double>(1,2);
    

    std::cout << "Using the follwing intrinsics matrix: " << std::endl << intrinsics << std::endl;



    // Initializing the MarkerDetector
    cv::runetag::MarkerDetector markerDetector( intrinsics );
    markerDetector.addModelsFromFile( "TestMarker.txt" );


    std::vector< RotatedRect > foundEllipses;
    runetag::EllipseDetector ellipseDetector( 10, 500, 100.0f, 5000.0f, 0.3f, 0.30f, -1.5);    
    ellipseDetector.detectEllipses( frame, foundEllipses );

    std::cout << foundEllipses.size() << " ellipses found" << std::endl;


    
    /**/
    std::vector< cv::runetag::EllipsePoint > ellipses;
    cv::Mat dbgframe = frame.clone();
    for( int i=0; i<foundEllipses.size(); ++i )
    {
        ellipses.push_back( cv::runetag::EllipsePoint(foundEllipses[i],cx,cy) );
        cv::runetag::AuxRenderer::drawEllipsePoint( dbgframe, ellipses.back(), intrinsics, CV_RGB(0,255,0));
    }
    cv::imwrite("ellipses.png", dbgframe );
    /**/


    std::vector< cv::runetag::MarkerDetected > tags_found;
    markerDetector.dbgimage = frame.clone();
    markerDetector.detectMarkers( foundEllipses, tags_found);


    //bool poseok;
    //cv::Mat distortion = cv::Mat::zeros(1,5,CV_32F);
    //cv::runetag::findPose( tags_found[0], intrinsics, distortion, poseok, 0);

    std::cout << "Press ENTER to exit" << std::endl;
    return 0;
}
#endif