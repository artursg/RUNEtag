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
#include "markerpose.hpp"
#include "ellipsepoint.hpp"
#include "digitalmarkermodel.hpp"
#include "auxmath.hpp"


using namespace cv::runetag;

bool dist_ordering( std::pair<float,unsigned int> a, std::pair<float, unsigned int> b ) 
{
    return ( a.first < b.first );
}


double avg_reprojection_error( const std::vector<cv::Point3f>& ref_points, 
							   const std::vector<cv::Point2f>& mark_points, 
							   const cv::Mat& Rvec, const cv::Mat& Tvec, const cv::Mat& intrinsics, const cv::Mat& distortion )
{
	std::vector< cv::Point2f > out_points;
    cv::projectPoints( ref_points, Rvec, Tvec, intrinsics, distortion, out_points );
	double avg_dist=0;
    double num_points = 0;
    for( size_t i=0; i<ref_points.size(); i++ ) 
    {
        const double distance = cv::norm( out_points[i]-mark_points[i] );
		avg_dist += distance;
		num_points+=1.0;
    }
	avg_dist=avg_dist/num_points;
	return avg_dist;
}

extern cv::runetag::Pose cv::runetag::findPose( const MarkerDetected& detected, const cv::Mat& intrinsics, const cv::Mat& distortion, bool* pose_ok, unsigned int method, unsigned int flag ) 
{
    cv::Mat Rvec( 1,3, CV_64F );
    cv::Mat Tvec( 1,3, CV_64F );
	cv::Mat Rvec_old( 1,3, CV_64F );
    cv::Mat Tvec_old( 1,3, CV_64F );

    std::vector< const cv::Matx33d* > model_ellipses;
    std::vector< const cv::Matx33d* > detec_ellipses;

    int off = detected.associatedModel()->getNumLayers();
    for( unsigned int i=0; i<detected.getNumSlots(); i++ ) 
    {
        if( detected.getSlot(i).value() && !detected.getSlot( (i+off)%detected.getNumSlots() ).discarded() && 
            !detected.getSlot( i-off<0?detected.getNumSlots()-off:i-off ).discarded() ) 
        {
            const cv::Matx33d* Sq = &detected.getSlot(i).getPayload()->getEllipse();
            const cv::Matx33d* Qi = &detected.associatedModel()->modelEllipseAtSlot(i);
            detec_ellipses.push_back( Sq );
            model_ellipses.push_back( Qi );
        }
    }

    const size_t N = model_ellipses.size();
   
	std::vector<cv::Point3f> ref_points( N );
    std::vector<cv::Point2f> mark_points( N );

    for( unsigned int i=0; i<N; i++ ) 
    {
        cv::Point2d model_center = ellipseCenter( *model_ellipses[i] );
        cv::Point2d detec_center = ellipseCenter( *detec_ellipses[i] );

        ref_points[i]= cv::Point3f( static_cast<float>( model_center.x ), static_cast<float>( model_center.y ), 0.0 );
        mark_points[i] = cv::Point2f( static_cast<float>( detec_center.x + intrinsics.at<double>(0,2)/*+0.5*/), static_cast<float>( detec_center.y + intrinsics.at<double>(1,2)/*+0.5*/  ) );
    }


    cv::solvePnP( ref_points, mark_points, intrinsics, distortion, Rvec, Tvec, false, method );
    if( pose_ok != 0 )
        *pose_ok=true;

	std::cout << "Avg. Reprojection error: " << avg_reprojection_error(ref_points,mark_points,Rvec,Tvec,intrinsics,distortion) << " px" << std::endl;

#if 0

//    if (flag == FLAG_REPROJ_ERROR || flag == ( FLAG_REPROJ_ERROR|FLAG_REFINE ) ) 
    // check if the first bit is set
    if ((static_cast<int>(flag)%2) == 1)
    {
        const double thresh = 1.5; //Reprojection error threshold
        bool *curr_set = new bool[ N ];
        for( unsigned int i=0; i<N; i++ )
            curr_set[i] = true;
        bool changed = true;
        int num_iterations = 1;

        while( changed ) 
        {
            changed=false;

            // Calc reprojection error
            std::vector< cv::Point2f > out_points;
            cv::projectPoints( ref_points, Rvec, Tvec, intrinsics, distortion, out_points );

            unsigned int num_points = 0;
            for( unsigned int i=0; i<N; i++ ) 
            {
                const double distance = (float)sqrt( (out_points[i].x - mark_points.at<float>(i,0))*(out_points[i].x - mark_points.at<float>(i,0)) +
                                                     (out_points[i].y - mark_points.at<float>(i,1))*(out_points[i].y - mark_points.at<float>(i,1)) );
                if( distance < thresh ) 
                {
                    if( curr_set[i] == false ) 
                    {
                        changed = true;
                    }
                    curr_set[i] = true;
                    num_points++;
                } 
                else 
                {
                    if( curr_set[i] == true ) 
                    {
                        changed = true;
                    }
                    curr_set[i] = false;
                }
            }

            if( num_points < 6 ) 
            {
                if( pose_ok != 0 )
                {
                    *pose_ok=false;
                }
                break;
            }
            // Estimate new pose with selected inliers
            std::vector< cv::Point3f > ref_points_refined;
            std::vector< cv::Point2f > mark_points_refined;
            for( unsigned int i=0; i<N; i++ ) 
            {
                if( curr_set[i] ) 
                {
                    ref_points_refined.push_back( ref_points[i]  );
                    mark_points_refined.push_back(mark_points[i] );
                }
            }

			std::cout << "PnP ransac " << num_iterations << " (" << num_points << " current inliers) ..." << std::endl;
            cv::solvePnP( cv::Mat(ref_points_refined), cv::Mat(mark_points_refined), intrinsics, distortion, Rvec, Tvec, false, flag );
            num_iterations++;
        }
        delete[] curr_set;
    }
#endif

    //if (flag == FLAG_REFINE || flag == ( FLAG_REPROJ_ERROR|FLAG_REFINE ) ) {
    // check if the second bit is set
    if ( (static_cast<int>(flag)/2)%2 == 1 )
    {
        // REFINE
        for( unsigned int iterations = 0; iterations<5; iterations++ ) 
        {
			cv::Matx33d R;
            cv::Rodrigues( Rvec, R );

            for( unsigned int kkk = 0; kkk < detec_ellipses.size(); kkk++ ) 
            {
                // COPYING detec_ellipses[kkk]
                cv::Matx33d el(*detec_ellipses[kkk]);

                double f = (intrinsics.at<double>(0,0) + intrinsics.at<double>(1,1)) / 2.0;
                el(0,2) /= f;
                el(1,2) /= f;
                el(2,0) /= f;
                el(2,1) /= f;
                el(2,2) /= f*f;

                cv::Matx33d el_cir =R.t()*el*R;

                cv::Point2f cir_center_p = ellipseCenter( el_cir );
                cv::Matx31d cir_center;
                cir_center(0,0) = cir_center_p.x;
                cir_center(1,0) = cir_center_p.y;
                cir_center(2,0) = 1.0;

                cv::Matx31d el_center = R*cir_center;
                el_center(0,0) /= el_center(2,0);
                el_center(1,0) /= el_center(2,0);
                el_center(2,0) = 1.0;

                mark_points[kkk] = cv::Point2f( static_cast<float>( el_center(0,0)*f + intrinsics.at<double>(0,2) ), static_cast<float>( el_center(1,0)*f + intrinsics.at<double>(1,2) ) );
            }

			Rvec_old = Rvec.clone();
			Tvec_old = Tvec.clone();
			std::cout << "PnP refine " << iterations << "..." << std::endl;
            cv::solvePnP( ref_points, mark_points, intrinsics, distortion, Rvec, Tvec, true, method );

			std::cout << " tvec delta norm: " << cv::norm(Tvec_old-Tvec) << std::endl;
			std::cout << " Rvec delta norm: " << cv::norm(Rvec_old-Rvec) << std::endl;			

			// Calc reprojection error
            
			std::cout << "Avg. Reprojection error: " << avg_reprojection_error(ref_points,mark_points,Rvec,Tvec,intrinsics,distortion) << " px" << std::endl;
        }
    // END REFINE
    }

    Pose outPose;
    outPose.R = Rvec;
    outPose.t = Tvec;
    return outPose;
}
