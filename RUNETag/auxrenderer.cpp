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

#include "auxrenderer.hpp"

#include "ellipsepoint.hpp"
#include "slot.hpp"
#include "imgtextstream.hpp"
#include "markerdetected.hpp"
#include "digitalmarkermodel.hpp"
#include "auxmath.hpp"


namespace cv { 
namespace runetag {

namespace AuxRenderer {


void drawEllipse( cv::Mat& frame, const cv::RotatedRect& el, const cv::Mat& intrinsics ) {
    try {
    cv::ellipse( frame, cv::Point2f( static_cast<float>(el.center.x + intrinsics.at<double>(0,2)), static_cast<float>(el.center.y+intrinsics.at<double>(1,2)) ), cv::Size( static_cast<int>(el.size.width/2), static_cast<int>(el.size.height/2) ), el.angle, 0.0, 360.0, CV_RGB(255,0,255), 1, CV_AA );
    } catch( cv::Exception& e ) {
        e.what();
    }
}


void drawPoint( cv::Mat& frame, const cv::Point2f p, const cv::Mat& intrinsics, cv::Scalar color ) {
    int i = static_cast<int>(p.y+intrinsics.at<double>(1,2));
    int j = static_cast<int>(p.x + intrinsics.at<double>(0,2));
    if( i<0 || i>= frame.rows || j<0 || j>=frame.cols )
        return;
    frame.at<unsigned char>( i, j*3 ) = static_cast<unsigned char>( color[0] );
    frame.at<unsigned char>( i, j*3+1 ) = static_cast<unsigned char>( color[1] );
    frame.at<unsigned char>( i, j*3+2 ) = static_cast<unsigned char>( color[2] );
}


void drawPoint( cv::Mat& frame, const cv::Point2f p, const cv::Mat& intrinsics ) {
	cv::circle( frame, cv::Point2f( static_cast<float>(p.x + intrinsics.at<double>(0,2)), static_cast<float>(p.y+intrinsics.at<double>(1,2)) ), 1, CV_RGB(255,0,255), 1 );
}


void drawEllipsePoint( cv::Mat frame, const EllipsePoint& e, const cv::Mat& intrinsics, cv::Scalar color ) {

	const double& A = e.getEllipse()(0,0);
	const double& B = e.getEllipse()(0,1);
	const double& C = e.getEllipse()(1,1);
	const double& D = e.getEllipse()(0,2);
	const double& E = e.getEllipse()(1,2);
	const double& F = e.getEllipse()(2,2);

	for( int xe=0; xe<frame.cols; xe++ ) {
        bool set=false;
		for( int ye=0; ye<frame.rows; ye++ ) {

			double x = xe-intrinsics.at<double>(0,2);
			double y = ye-intrinsics.at<double>(1,2);

			//Calc value
			double v = A*x*x + 2.0*B*x*y + C*y*y + 2.0*D*x + 2.0*E*y + F;
			if( !set && v  < 0.0 ) {
                set=true;
                AuxRenderer::drawPoint( frame, cv::Point2f( (float)x, (float)y ), intrinsics, color );
			}

            if( set && v  > 0.0 ) {
                set=false;
				AuxRenderer::drawPoint( frame, cv::Point2f( (float)x, (float)y ), intrinsics, color );
			}
		}
	}
}


void drawEllipse( cv::Mat frame, const cv::Mat& e, const cv::Mat& intrinsics, cv::Scalar color ) {

	const double& A = e.at<double>(0,0);
	const double& B = e.at<double>(0,1);
	const double& C = e.at<double>(1,1);
	const double& D = e.at<double>(0,2);
	const double& E = e.at<double>(1,2);
	const double& F = e.at<double>(2,2);

	for( int xe=0; xe<frame.cols; xe++ ) {
        bool set=false;
		for( int ye=0; ye<frame.rows; ye++ ) {

			double x = xe-intrinsics.at<double>(0,2);
			double y = ye-intrinsics.at<double>(1,2);

			//Calc value
			double v = A*x*x + 2.0*B*x*y + C*y*y + 2.0*D*x + 2.0*E*y + F;
			if( !set && v  < 0.0 ) {
                set=true;
                AuxRenderer::drawPoint( frame, cv::Point2f( (float)x, (float)y ), intrinsics, color );
			}

            if( set && v  > 0.0 ) {
                set=false;
				AuxRenderer::drawPoint( frame, cv::Point2f( (float)x, (float)y ), intrinsics, color );
			}
		}
	}
}


void fillEllipse( cv::Mat frame, const cv::Mat& e, const cv::Mat& intrinsics, cv::Scalar color ) {

	const double& A = e.at<double>(0,0);
	const double& B = e.at<double>(0,1);
	const double& C = e.at<double>(1,1);
	const double& D = e.at<double>(0,2);
	const double& E = e.at<double>(1,2);
	const double& F = e.at<double>(2,2);

	for( int xe=0; xe<frame.cols; xe++ ) {
		for( int ye=0; ye<frame.rows; ye++ ) {

			double x = xe-intrinsics.at<double>(0,2);
			double y = ye-intrinsics.at<double>(1,2);

			//Calc value
			double v = A*x*x + 2.0*B*x*y + C*y*y + 2.0*D*x + 2.0*E*y + F;
			if( v  < 0.0 ) {
                AuxRenderer::drawPoint( frame, cv::Point2f( (float)x, (float)y ), intrinsics, color );
			}
		}
	}
}


void fillSlot( cv::Mat& frame, const Slot& slot, cv::Scalar color, const cv::Mat& intrinsics ) {

	for( int xe=0; xe<frame.cols; xe++ ) {
		for( int ye=0; ye<frame.rows; ye++ ) {

			float x = static_cast<float>( xe-intrinsics.at<double>(0,2) );
			float y = static_cast<float>( ye-intrinsics.at<double>(1,2) );
			if( slot.checkInside( cv::Point2f( x, y) ) ) {
                AuxRenderer::drawPoint( frame, cv::Point2f( (float)x, (float)y ), intrinsics, color );
			}
		}
	}

}


void drawDetectedMarker( cv::Mat& frame, const MarkerDetected& m, const cv::Mat& intrinsics ) {

    /*
    cv::Point2f pos( static_cast<float>(slot0.getCenter().x + intrinsics.at<double>(0,2)), 
                     static_cast<float>(slot0.getCenter().y + intrinsics.at<double>(1,2)) );
    cvExt::its( frame ) << cvExt::its::position( pos ) << m.associatedModel()->name;

    */

    /*
    for( unsigned int i=0; i<m.getNumSlots(); i++ ) 
    {
        if( m.getSlot(i).value() ) {
        cv::Point2f pos( static_cast<float>( m.getSlot(i).getCenter().x + intrinsics.at<double>(0,2)), 
            static_cast<float>( m.getSlot(i).getCenter().y + intrinsics.at<double>(1,2)) );
        
        std::cout << "POS: " << pos << std::endl;
        cv::runetag::its( frame ) << cv::runetag::its::position( pos ) << i;
        }
    }
    */
    

#if 1
    for( unsigned int i=0; i<m.getNumSlots(); i++ ) 
    {
        //if( m.getSlot(i).value() ) 
        {
            /*
            cv::Point2f c = m.getSlot(i).getPayload()->getCenter();
            cv::Point2f pos( static_cast<float>(c.x + intrinsics.at<double>(0,2)), 
                     static_cast<float>(c.y + intrinsics.at<double>(1,2)) );*/
            cv::Point2f c = m.getSlot(i).getCenter();
            cv::Point2f pos( static_cast<float>(c.x + intrinsics.at<double>(0,2)), 
                     static_cast<float>(c.y + intrinsics.at<double>(1,2)) );
        	
            //its stream = its(frame);
        	//stream << its::position( pos ) << i;
        	// 
        	if( m.getSlot(i).discarded() ) 
                cv::circle(frame, pos, 5, CV_RGB(255,0,0),CV_FILLED, CV_AA);
            else if( m.getSlot(i).value() ) 
            {
                cv::circle(frame, pos, 5, CV_RGB(0,255,0),CV_FILLED, CV_AA);
            }
            else
            {
                cv::circle(frame, pos, 5, CV_RGB(255,0,0),1, CV_AA);
            }
        }
    }
#endif
#if 0
    int i=0;
    {
        
        cv::Point2f c = m.getSlot(i).getPayload()->getCenter();
        cv::Point2f pos( static_cast<float>(c.x + intrinsics.at<double>(0,2)), 
            static_cast<float>(c.y + intrinsics.at<double>(1,2)) );
        its stream = its(frame);
        stream << its::position( pos ) << i;
    }
    {

        cv::Point2f c = m.getSlot(i).slot_center;
        cv::Point2f pos( static_cast<float>(c.x + intrinsics.at<double>(0,2)), 
            static_cast<float>(c.y + intrinsics.at<double>(1,2)) );
        cv::circle(frame, pos, 5, CV_RGB(255,0,0),CV_FILLED, CV_AA);
    }    
#endif 
    

//    drawEllipsePoint( frame, m.e1, intrinsics, CV_RGB(0,0,255) );
//    drawEllipsePoint( frame, m.e2, intrinsics, CV_RGB(0,255,0) );
}


void drawDetectedMarker3D( cv::Mat& frame, const MarkerDetected& m, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion, cv::Scalar color ) {

    float model_rad = static_cast<float>( m.associatedModel()->getWorldSize() );

    std::vector< cv::Point2f > out_points;
					
	cv::Mat points( 9,3,CV_32F );
	points.at<float>(0,0) =  -model_rad; points.at<float>(0,1) = -model_rad; points.at<float>(0,2) = 0.0;
	points.at<float>(1,0) = model_rad; points.at<float>(1,1) =  -model_rad;points.at<float>(1,2) = 0.0;
	points.at<float>(2,0) = model_rad; points.at<float>(2,1) = model_rad;points.at<float>(2,2) = 0.0;
	points.at<float>(3,0) =   -model_rad; points.at<float>(3,1) = model_rad;points.at<float>(3,2) = 0.0;

	points.at<float>(4,0) =   -model_rad; points.at<float>(4,1) =  -model_rad; points.at<float>(4,2) = -model_rad;
	points.at<float>(5,0) = model_rad; points.at<float>(5,1) =   -model_rad;points.at<float>(5,2) = -model_rad;
	points.at<float>(6,0) = model_rad; points.at<float>(6,1) = model_rad;points.at<float>(6,2) = -model_rad;
	points.at<float>(7,0) =   -model_rad; points.at<float>(7,1) = model_rad;points.at<float>(7,2) = -model_rad;

    points.at<float>(8,0) =  0.0f; points.at<float>(8,1) = 0.0f; points.at<float>(8,2) = 0.0f;

    cv::projectPoints( points, p.R, p.t, intrinsics, distortion, out_points );
	
	for( unsigned int i=0; i<8; i++ ) {
		for( unsigned int j=i+1; j<8; j++ ) {
            cv::Point2f source = out_points[i];
            cv::Point2f dest = out_points[j];
			cv::line( frame, source, dest, color, 1, 4  );
		}
	}
	its stream = its(frame);
	stream << its::position(out_points[8].x, out_points[8].y) << m.associatedModel()->getName();
}


//
//  CVPR
//
void drawDetectedMarker3DSlots( cv::Mat& frame, const MarkerDetected& m, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion, cv::Scalar color ) {

    float model_rad = static_cast<float>( m.associatedModel()->getWorldSize() );
    const unsigned int num_slots = m.associatedModel()->getNumSlots() / m.associatedModel()->getNumLayers();
    double angle = 6.283185 / (double)num_slots;

    std::vector< cv::Point2f > out_points;
    cv::Mat points( num_slots+1, 3, CV_32F );
    points.at<float>(0,0) = 0.0; points.at<float>(0,1) = 0.0; points.at<float>(0,2) = 0.0;

    std::vector< cv::Point2f > out_points_outer;
    cv::Mat points_outer( num_slots, 3, CV_32F );

    for( unsigned int i=1; i<num_slots+1; i++ ) {
        double curr_angle = angle*(i-1);
        points.at<float>(i,0) = (float)(model_rad*cos(curr_angle+angle/2.0)); 
        points.at<float>(i,1) = (float)(model_rad*sin(curr_angle+angle/2.0));
        points.at<float>(i,2) = 0.0f;

        points_outer.at<float>(i-1,0) = points.at<float>(i,0)*1.1f;
        points_outer.at<float>(i-1,1) = points.at<float>(i,1)*1.1f;
        points_outer.at<float>(i-1,2) = 0.0f;
    }

    //Project points
    cv::projectPoints( points, p.R, p.t, intrinsics, distortion, out_points );
    cv::projectPoints( points_outer, p.R, p.t, intrinsics, distortion, out_points_outer );

    for( unsigned int i=1; i<num_slots+1; i++ ) {
        cv::line( frame, out_points[0], out_points_outer[i-1], color, 1, CV_AA  );
    }

    cv::RotatedRect fit = cv::fitEllipse( cv::Mat( out_points_outer ) );
    fit.size.width /= 2.0f;
    fit.size.height /= 2.0f;
    fit.angle = 180.0f-fit.angle;

    //cv::ellipse( frame, fit.center, fit.size, fit.angle, 0, 360, CV_RGB(0,255,0),1 );

    
    float gap = 0.8f;
    cv::RotatedRect inner = fit;
    inner.size.width *= gap;
    inner.size.height *= gap;
    //cv::ellipse( frame, inner.center, inner.size, inner.angle, 0, 360, CV_RGB(20,150,30),1,CV_AA  );
    cv::ellipse( frame, fit.center, fit.size, fit.angle, 0, 360, CV_RGB(20,150,30),1,CV_AA  );

}



void drawDetectedMarker3DCylinder( cv::Mat& frame, const MarkerDetected& m, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion, cv::Scalar color ) {

    const double radius = 1.0 * (  (m.associatedModel()->getNumLayers()+1)/(double)(m.associatedModel()->getNumLayers()*2.0)  );

    float model_rad = static_cast<float>(m.associatedModel()->getWorldSize() * radius);
    const unsigned int num_slots = m.associatedModel()->getNumSlots();
    const unsigned int num_symbols = m.associatedModel()->numSymbolsInternal();
    double angle = 6.283185 / (double)num_slots;

    std::vector< cv::Point2f > out_points;
    cv::Mat points( num_symbols, 3, CV_32F );

    unsigned int k=0;
    unsigned int j=0;
    for( unsigned int i=0; i<num_slots; i++ ) {
        double curr_angle = angle*(k);
        if( m.associatedModel()->isInternal(k) && m.associatedModel()->valueAtSlot( k ) ) {
            points.at<float>(j,0) = (float)(model_rad*cos(curr_angle)); 
            points.at<float>(j,1) = (float)(model_rad*-sin(curr_angle));
            points.at<float>(j,2) = 0.0f;

            points.at<float>(j+1,0) = points.at<float>(j,0); 
            points.at<float>(j+1,1) = points.at<float>(j,1);
            points.at<float>(j+1,2) = -model_rad;
            j+=2;
        }
        k++;
    }

    //Project points
    cv::projectPoints( points, p.R, p.t, intrinsics, distortion, out_points );

    for( unsigned int i=0; i<num_symbols; i+=2 ) {
        unsigned int inext = (i+1)%(num_symbols);
        unsigned int inext2 = (i+2)%(num_symbols);
        unsigned int inext3 = (i+3)%(num_symbols);
        cv::line( frame, out_points[i], out_points[inext], color, 1, CV_AA  );

        cv::line( frame, out_points[i], out_points[inext2], color, 1, CV_AA  );
        cv::line( frame, out_points[inext], out_points[inext3], color, 1, CV_AA  );

    }


    for( unsigned int i=0; i<m.getNumSlots(); i++ ) {
        if( m.getSlot(i).discarded() && m.getSlot(i).getPayload() ) {
            cv::Point2d p = m.getSlot(i).getPayload()->getCenter();
            p.x += intrinsics.at<double>(0,2);
            p.y += intrinsics.at<double>(1,2);
            cv::circle( frame, p, 2, CV_RGB(0,255,0), 2 );
        }
    }

}



void drawDetectedMarker3DCylinder2( cv::Mat& frame, const MarkerDetected& m, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion ) {

    std::vector< cv::Point3f> bottom_vertices;
    std::vector< cv::Point3f> top_vertices;

    std::vector< cv::Point3f > marker_dots;
    std::vector< bool > valid_slots;

    std::vector< cv::Point3f > marker_center(2);
    
    float model_rad = static_cast<float>( m.associatedModel()->getWorldSize() );
    for( MarkerDetected::SlotIterator it=m.begin_by_layer(0); it!=m.end_by_layer(0); ++it )
    {
        try
        {
            cv::Point2d ptcenter = cv::runetag::ellipseCenter( m.associatedModel()->modelEllipseAtSlot(it.slotIDX()) );            

            bottom_vertices.push_back( cv::Point3f((float)ptcenter.x, (float)ptcenter.y, 0 ) );
            top_vertices.push_back( bottom_vertices.back()+cv::Point3f(0,0,-model_rad/2.0f) );

        }catch( cv::runetag::DigitalMarkerModel::MarkerModelOperationException )
        {
        }
    }

    for( unsigned int i=0; i<m.getNumSlots(); ++i )
    {
        try
        {
            cv::Point2d ptcenter = cv::runetag::ellipseCenter( m.associatedModel()->modelEllipseAtSlot(i) );
            marker_dots.push_back( cv::Point3f((float)ptcenter.x, (float)ptcenter.y, 0 ) );
            valid_slots.push_back( m.getSlot(i).value() );
        }catch( cv::runetag::DigitalMarkerModel::MarkerModelOperationException )
        {
        }
    }

    marker_center[1].x = model_rad;

    //Project points
    std::vector< cv::Point2f> bottom_vertices_img;
    std::vector< cv::Point2f> top_vertices_img;
    std::vector< cv::Point2f > marker_dots_img;
    std::vector< cv::Point2f > marker_center_img;

    cv::projectPoints( bottom_vertices, p.R, p.t, intrinsics, distortion, bottom_vertices_img );
    cv::projectPoints( top_vertices, p.R, p.t, intrinsics, distortion, top_vertices_img );
    cv::projectPoints( marker_dots, p.R, p.t, intrinsics, distortion, marker_dots_img );
    cv::projectPoints( marker_center, p.R, p.t, intrinsics, distortion, marker_center_img );
    
    // Render cylinder
    cv::Scalar color = CV_RGB(0,50,255);
    for( size_t i=0; i<bottom_vertices_img.size(); ++i )
    {
        cv::line( frame, bottom_vertices_img[i], top_vertices_img[i], color, 2, CV_AA );
        cv::line( frame, bottom_vertices_img[i], bottom_vertices_img[(i+1)%bottom_vertices_img.size()], color, 2, CV_AA );
        cv::line( frame, top_vertices_img[i], top_vertices_img[(i+1)%top_vertices_img.size()], color, 2, CV_AA );
    }

    // Render marker dots
    for( size_t i=0; i<marker_dots_img.size(); ++i )
    {
        cv::circle(frame, marker_dots_img[i],3,valid_slots[i]?CV_RGB(0,255,0):CV_RGB(255,0,0),-1,CV_AA );
    }


    cv::line( frame, marker_center_img[0], marker_center_img[1], color, 2, CV_AA );
    its stream = its(frame);
    stream << its::position(marker_center_img[0].x, marker_center_img[0].y) << m.associatedModel()->getName();
}


//
//  CVPR
//
void drawDetectedMarker3Dfits( cv::Mat& frame, const MarkerDetected& m, unsigned int slot1, unsigned int slot2, unsigned int real_slot1, unsigned int real_slot2, const Pose& p, const cv::Mat& intrinsics, const cv::Mat& distortion ) {

    float model_rad = static_cast<float>( m.associatedModel()->getWorldSize() );
    const unsigned int num_slots = m.associatedModel()->getNumSlots() / m.associatedModel()->getNumLayers();
    double angle = 6.283185 / (double)num_slots;

    std::vector< cv::Point2f > out_points;
    cv::Mat points( num_slots, 3, CV_32F );

    for( unsigned int i=0; i<num_slots; i++ ) {
        double curr_angle = angle*i;
        points.at<float>(i,0) = (float)(model_rad*cos( curr_angle )); 
        points.at<float>(i,1) = (float)(model_rad*-sin( curr_angle ));
        points.at<float>(i,2) = 0.0f;
    }

    std::vector< cv::Point2f > out_points_reflected;
    cv::Mat points_reflected( num_slots, 3, CV_32F );
    cv::Vec2f A( points.at<float>(slot1,0), points.at<float>(slot1,1) );
    cv::Vec2f B( points.at<float>(slot2,0), points.at<float>(slot2,1) );
    cv::Vec2f v0 = B-A;
    const float v0len = (float)cv::norm(v0);
    for( unsigned int i=0; i<num_slots; i++ ) {
        points_reflected.at<float>(i,2)=0.0;
        if( i==slot1 ) {
            points_reflected.at<float>(i,0) = points.at<float>(slot1,0);
            points_reflected.at<float>(i,1) = points.at<float>(slot1,1);
            continue;
        }
        if( i==slot2 ) {
            points_reflected.at<float>(i,0) = points.at<float>(slot2,0);
            points_reflected.at<float>(i,1) = points.at<float>(slot2,1);
            continue;
        }

        cv::Vec2f C( points.at<float>(i,0), points.at<float>(i,1) );
        cv::Vec2f v1 = C-A;
        double k = (v1[0]*v0[0] + v1[1]*v0[1])/v0len;
        cv::Vec2f D;
        D[0] = static_cast<float>(v0[0]/v0len*k);
        D[1] = static_cast<float>(v0[1]/v0len*k);
        cv::Vec2f O = A+D;

        cv::Vec2f Z = C-O;

        cv::Vec2f C1 = O-Z;

        //std::cout << k << std::endl;
        //cv::line( frame, cv::Point2f( C[0], C[1] ), cv::Point2f( C1[0], C1[1] ), CV_RGB(0,0,0),1,CV_AA);
        //cv::circle( frame, cv::Point2f(O[0], O[1]), 2, CV_RGB(0,0,0),2 );
        //cv::circle( frame, cv::Point2f(C1[0], C1[1]), 2, CV_RGB(0,0,0),2 );
        points_reflected.at<float>(i,0) = C1[0];
        points_reflected.at<float>(i,1) = C1[1];
    }


    //Project points
    cv::projectPoints( points, p.R, p.t, intrinsics, distortion, out_points );
    cv::projectPoints( points_reflected, p.R, p.t, intrinsics, distortion, out_points_reflected );

   
    if( m.getSlot(real_slot1).value() ) {
		cv::Matx33d el = m.getSlot(real_slot1).getPayload()->getEllipse();
        cv::RotatedRect rr = ellipseToRotatedRect( el );
        rr.center.x += static_cast<float>(intrinsics.at<double>(0,2));
        rr.center.y += static_cast<float>(intrinsics.at<double>(1,2));
        rr.angle = rr.angle;
        cv::ellipse( frame, rr.center, rr.size, rr.angle, 0, 360, CV_RGB( 0,255,0 ),1, CV_AA );
    }

    if( m.getSlot(real_slot2).value() ) {
		cv::Matx33d el = m.getSlot(real_slot2).getPayload()->getEllipse();
        cv::RotatedRect rr = ellipseToRotatedRect( el );
        rr.center.x += static_cast<float>(intrinsics.at<double>(0,2));
        rr.center.y += static_cast<float>(intrinsics.at<double>(1,2));
        rr.angle = rr.angle;
        cv::ellipse( frame, rr.center, rr.size, rr.angle, 0, 360, CV_RGB( 255,255,0 ),1, CV_AA );
    }
    
    cv::RotatedRect fit1 = cv::fitEllipse( cv::Mat( out_points ) );
    cv::RotatedRect fit2 = cv::fitEllipse( cv::Mat( out_points_reflected ) );

    fit1.size.width /= 2.0;
    fit1.size.height /= 2.0;
    fit2.size.width /= 2.0;
    fit2.size.height /= 2.0;
    fit1.angle = 180.0f-fit1.angle;
    fit2.angle = 180.0f-fit2.angle;

    cv::ellipse( frame, fit1.center, fit1.size, fit1.angle, 0, 360, CV_RGB( 0,0,255 ),1, CV_AA );
    cv::ellipse( frame, fit2.center, fit2.size, fit2.angle, 0, 360, CV_RGB( 0,0,255 ),1, CV_AA );  
    
}



void drawVector( cv::Mat& frame, const cv::Point2f center, const cv::Point2f vector, cv::Scalar color, const cv::Mat& intrinsics ) {
	cv::line( frame, cv::Point2f( static_cast<float>(center.x + intrinsics.at<double>(0,2)), static_cast<float>(center.y + intrinsics.at<double>(1,2))), cv::Point2f( static_cast<float>(vector.x + center.x + intrinsics.at<double>(0,2)), static_cast<float>(vector.y + center.y + intrinsics.at<double>(1,2)) ), color, 1 );
}

void drawLine( cv::Mat& frame, const cv::Point2f p1, const cv::Point2f p2, cv::Scalar color, const cv::Mat& intrinsics ) {
	cv::line( frame, 
			  cv::Point2f( static_cast<float>(p1.x + intrinsics.at<double>(0,2)), static_cast<float>(p1.y + intrinsics.at<double>(1,2)) ), 
			  cv::Point2f( static_cast<float>(p2.x + intrinsics.at<double>(0,2)), static_cast<float>(p2.y + intrinsics.at<double>(1,2)) ), 
			  color, 1 );
}



}

}
}