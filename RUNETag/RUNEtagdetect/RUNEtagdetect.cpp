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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/videoio/videoio_c.h>

#include <boost/program_options.hpp>
#include <iostream>
#include "runetag.hpp"
#include "auxrenderer.hpp"
#include "ellipsefitter.hpp"

#include "markerpose.hpp"
#include "coding.h"



#define ENABLE_TESTS


#ifdef ENABLE_TESTS
#include "test_occlusion.h"
#endif

using namespace cv;

namespace po = boost::program_options;






int main (int argc, char** argv)
{

    std::cout << "█▀█ █░█ █▄░█ █▀▀ ▀█▀ ▄▀█ █▀▀   █▀▄ █▀▀ ▀█▀ █▀▀ █▀▀ ▀█▀ █▀█ █▀█" << std::endl;
    std::cout << "█▀▄ █▄█ █░▀█ ██▄ ░█░ █▀█ █▄█   █▄▀ ██▄ ░█░ ██▄ █▄▄ ░█░ █▄█ █▀▄" << std::endl;
    std::cout << std::endl;



    /************************************************************************/
    /*         Settings & Data                                              */
    /************************************************************************/
    unsigned short min_ellipse_contour_points=10;
    unsigned short max_ellipse_contour_points=10000;
    float min_ellipse_area=100.0f;
    float max_ellipse_area=10000.0f;
    float min_roundness=0.3f;
    float max_mse = 0.3f;
    float size_compensation=-1.5f;
	bool dorefine;
	float gauss_smooth_sigma;
	int sobel_winsize;

#ifdef ENABLE_TESTS
	bool doocclusiontest;
	std::string occlusion_test_filename;
	int num_occlusion_tests;
	bool occlusion_test_interactive;
#endif
    
    cv::Mat input_image;
    std::string ellipseimg_filename;
    std::string tagsimg_filename;
    std::string points_filename;
    std::string pose_filename;

	bool pnpransac;
	bool pnprefine;
	unsigned int method;

    cv::runetag::MarkerDetector* pDetector=0;
    cv::Mat intrinsics;


    /************************************************************************/
    /*         Program Options setup                                        */
    /************************************************************************/
    po::options_description input_opts("Input");
    input_opts.add_options()
        ("config",po::value< std::string >(),"config filename")
        ("dir",po::value< std::string >(),"data directory")  
        ("img",po::value< std::string >(),"image filename")
        ("m",po::value< std::vector<std::string> >(),"tag model file (this option can be repeated many times)")
        ("f",po::value< double >(),"focal length (both fx and fy)")
        ("fx",po::value< double >(),"focal length x")
        ("fy",po::value< double >(),"focal length y")
        ("cx",po::value< double >(),"optical center x")
        ("cy",po::value< double >(),"optical center y")
		("intrinsicsfile",po::value< std::string >(),"intrinsics matrix (opencv format)")
		("distortionfile",po::value< std::string >(),"distortion matrix (opencv format)")
#ifdef ENABLE_TESTS
		("occlusion_test",po::bool_switch(&doocclusiontest),"Perform the occlusion test")
		("occlusion_test_filename",po::value< std::string >(&occlusion_test_filename),"Occlusion test results filename")
		("occlusion_test_n",po::value< int >(&num_occlusion_tests)->default_value( 10 ),"Number of occlusion tests to perform")
		("occlusion_test_interactive",po::bool_switch(&occlusion_test_interactive),"View occlusion test working")
#endif
        ;

    po::options_description output_opts("Output");
    output_opts.add_options()
        ("ellipseimg",po::value< std::string >(),"Render detected ellipses to image file")
        ("tagsimg",po::value< std::string >(),"Render detected tags to image file")
        ("pointsfile",po::value< std::string >(),"Detected tags points filename")
        ("posefile",po::value< std::string >(),"Detected tags pose filename")
        ;

    po::options_description ellipse_opts("Ellipse detector");
    ellipse_opts.add_options()
        ("minarea",po::value< float >(),"Minimum ellipse area [100]")
        ("maxarea",po::value< float >(),"Maximum ellipse area [10000]")
        ("minroundness",po::value< float >(),"Minimum ellipse roundness [0.3]")
        ("maxmse",po::value< float >(),"Maxmimum ellipse contour error [0.3]")
		("refine",po::bool_switch(&dorefine),"Refine ellipses")
		("pnpmethod",po::value<unsigned int>(&method)->default_value(0),"PnP Method: 0=Iterative, 1=ePnP")
		("pnpransac",po::bool_switch(&pnpransac),"Use robust ransac estimation for PnP")
		("pnprefine",po::bool_switch(&pnprefine),"Refine ellipse centers with PnP")
		("refine_gauss_smooth_sigma",po::value< float >(&gauss_smooth_sigma)->default_value( 3.0 ),"Gaussian blur sigma to be applied before sobel")
		("sobel_winsize",po::value< int >(&sobel_winsize)->default_value( 3 ),"Sobel window size")
        ;


    bool load_config_file = false;
    
    try {


        std::string config_filename;
        {
            // Check if "config" option is set in command line arguments
            po::options_description cmdline_options;
            cmdline_options.add(input_opts).add(output_opts).add(ellipse_opts);
            po::variables_map varmap;
            po::store( po::parse_command_line(argc, argv, cmdline_options), varmap );
            po::notify(varmap);
            if( varmap.count("config") )
            {
                config_filename = varmap["config"].as< std::string >();
                load_config_file = true;
            }
        }


        po::options_description cmdline_options;
        cmdline_options.add(input_opts).add(output_opts).add(ellipse_opts);
        po::variables_map varmap;

        if( load_config_file )
        {
            std::ifstream ifs( config_filename.c_str() );
            if( ifs.fail() )
            {
                std::cout << "Unable to open " << config_filename << std::endl;
            } else 
            {
                po::store( po::parse_config_file(ifs, cmdline_options ), varmap );
                ifs.close();
            }
        }
        po::store( po::parse_command_line(argc, argv, cmdline_options), varmap );
        po::notify(varmap);


        if (argc == 1 || varmap.count("help")) 
        {
            std::cout << cmdline_options << "\n";
            return 1;
        }


        /************************************************************************/
        /*  Process input arguments                                             */
        /************************************************************************/

        std::string workdir;
        if( varmap.count("dir") )
        {
            workdir = varmap["dir"].as<std::string>();
            if( workdir.length()>0 && workdir[workdir.length()-1]!='\\' && workdir[workdir.length()-1]!='/' )
            {
                workdir = workdir + std::string("\\");
            }
            std::cout << "+ Workdir set to: " << std::endl << workdir << std::endl << std::endl;
        }


        std::string img_filename;
        if( varmap.count("img") )
        {
            img_filename = varmap["img"].as<std::string>();
            if( workdir.length()>0 )
            {
                img_filename = workdir + img_filename;
            }
            std::cout << "+ Image filename: " << std::endl << img_filename << std::endl << std::endl;
        }
        else
        {
            std::cout << "Error: you must specify image filename" << std::endl;
            return -1;
        }

        // Read input image
        input_image = cv::imread( img_filename );
        if( input_image.rows==0 || input_image.cols==0 )
        {
            std::cout << "Error: unable to open input image" << std::endl;
            return -1;
        }

		
        // Input calibration
        intrinsics = cv::Mat::eye(3,3,CV_64FC1);

		
		if( varmap.count("intrinsicsfile") )
        {
			std::string intrinsics_filename = varmap["intrinsicsfile"].as<std::string>();
			try {
				// intrinsics = cv::cvarrToMat( cvLoad( intrinsics_filename.c_str() ), true );				
			} catch( cv::Exception e ) {
				std::cout << "Unable to load " << intrinsics_filename << ", aborting" << std::endl;
				return -1;
			}
		} 
		else 
		{
			if( varmap.count("f") )
			{
				intrinsics.at<double>(0,0) = intrinsics.at<double>(1,1) = varmap["f"].as<double>();        
			}
			if( varmap.count("fx") )
			{
				intrinsics.at<double>(0,0) = varmap["fx"].as<double>();        
			}
			if( varmap.count("fy") )
			{
				intrinsics.at<double>(1,1) = varmap["fy"].as<double>();             
			}
			if( varmap.count("cx") )
			{
				intrinsics.at<double>(0,2) = varmap["cx"].as<double>();        
			} else 
			{
				std::cout << "- cx not set, assuming image.cols/2" << std::endl;
				intrinsics.at<double>(0,2) = input_image.cols/2.0;
			}
			if( varmap.count("cy") )
			{
				intrinsics.at<double>(1,2) = varmap["cy"].as<double>();             
			} else 
			{
				std::cout << "- cy not set, assuming image.rows/2" << std::endl;
				intrinsics.at<double>(1,2) = input_image.rows/2.0;
			}
		}

        
        std::cout << "+ Camera intrinsics: " << std::endl;
        std::cout << "     fx: " << intrinsics.at<double>(0,0) << std::endl;
        std::cout << "     fy: " << intrinsics.at<double>(1,1) << std::endl;
        std::cout << "     cx: " << intrinsics.at<double>(0,2) << std::endl;
        std::cout << "     cy: " << intrinsics.at<double>(1,2) << std::endl << std::endl;


		cv::Mat distortion;
		if( varmap.count("distortionfile") )
        {
			std::string distortion_filename = varmap["distortionfile"].as<std::string>();
			try {
				//distortion = cv::cvarrToMat( cvLoad( distortion_filename.c_str() ), true );
				
				std::cout << "+ Distortion coefficients: " << std::endl;
				std::cout << "    " << distortion << std::endl;
			} catch( cv::Exception e ) {
				std::cout << "Unable to load " << distortion_filename << ", skipping" << std::endl;
			}
		}

		if( distortion.rows!=0 && distortion.cols!=0 )
		{
			std::cout << "+ Undistorting input image" << std::endl;
			cv::Mat input_image_undistorted;
			cv::undistort(input_image, input_image_undistorted, intrinsics, distortion );
			input_image = input_image_undistorted;
		}

        pDetector = new cv::runetag::MarkerDetector( intrinsics );        
        
        if( varmap.count("m") )
        {
            std::vector< std::string > models = varmap["m"].as< std::vector<std::string> >();

            std::cout << "+ Loading models: " << std::endl;
            for( std::vector< std::string >::const_iterator it=models.begin(); it!=models.end(); ++it )
            {
                std::string model_name = workdir + *it;
                
                std::cout << "  " << model_name << " ";

                try 
                {
                    std::cout << " " << pDetector->addModelsFromFile( model_name ) <<  " loaded" << std::endl;
                } catch( cv::runetag::DigitalMarkerModel::MarkerModelLoadException& ex ) 
                {
                    std::cout << std::endl << ex.what() << std::endl;
                    return -1;
                }
            }
            std::cout << std::endl;


        } else 
        {
            std::cout << "Error: you must specify at least one model file" << std::endl;
            return -1;
        }
        


        if( varmap.count("ellipseimg") )
        {
            ellipseimg_filename = varmap["ellipseimg"].as<std::string>();
            if( workdir.length()>0 )
            {
                ellipseimg_filename = workdir + ellipseimg_filename;
            }
        }

        if( varmap.count("tagsimg") )
        {
            tagsimg_filename = varmap["tagsimg"].as<std::string>();
            if( workdir.length()>0 )
            {
                tagsimg_filename = workdir + tagsimg_filename;
            }
        }

        if( varmap.count("pointsfile") )
        {
            points_filename = varmap["pointsfile"].as<std::string>();
            if( workdir.length()>0 )
            {
                points_filename = workdir + points_filename;
            }
        }

        if( varmap.count("posefile") )
        {
           pose_filename = varmap["posefile"].as<std::string>();
            if( workdir.length()>0 )
            {
                pose_filename = workdir + pose_filename;
            }
        }

        if( varmap.count("minarea") )
        {
            min_ellipse_area = varmap["minarea"].as<float>();
        }

        if( varmap.count("maxarea") )
        {
            max_ellipse_area = varmap["maxarea"].as<float>();
        }

        if( varmap.count("minroundness") )
        {
            min_roundness = varmap["minroundness"].as<float>();
        }

        if( varmap.count("maxmse") )
        {
            max_mse = varmap["maxmse"].as<float>();
        }
        

    } catch( boost::program_options::error&  e )
    {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    }



    // Detect ellipses
    std::vector< RotatedRect > foundEllipses;
    runetag::EllipseDetector ellipseDetector( min_ellipse_contour_points, 
        max_ellipse_contour_points, 
        min_ellipse_area,
        max_ellipse_area, 
        min_roundness, 
        max_mse, 
        size_compensation);    


    std::cout << "> Detecting ellipses" << std::endl;
    ellipseDetector.detectEllipses( input_image, foundEllipses );
    std::cout << "  " << foundEllipses.size() << " found." << std::endl << std::endl;

    if( ellipseimg_filename.length()>0 )
    {
        // Debug ellipses
        cv::Mat dbgout = input_image.clone();

        for( size_t i=0; i<foundEllipses.size(); ++i )
        {
            cv::RotatedRect rr = foundEllipses[i];
            rr.angle=-rr.angle;
            cv::ellipse( dbgout, rr, CV_RGB(255,0,0),1,cv::LINE_AA);
        }

        std::cout << "> Rendering ellipses" << std::endl;
        std::cout << "   -> " << ellipseimg_filename << std::endl << std::endl;
        cv::imwrite( ellipseimg_filename, dbgout );
    }


    std::cout << "> Detecting RUNE tags" << std::endl;
    std::vector< cv::runetag::MarkerDetected > tags_found;
    pDetector->dbgimage = input_image.clone();
    pDetector->detectMarkers( foundEllipses, tags_found);
    std::cout << "  " << tags_found.size() << " found." << std::endl << std::endl;



	if( dorefine )
	{
		std::cout << "> Refining ellipses" << std::endl;
		
		if( sobel_winsize%2==0 )
		{
			sobel_winsize++;
			std::cout << " Sobel winsize changed to " << sobel_winsize << " (must be odd)" << std::endl;
		}

		// Sobel
		cv::Mat input_image_gray;
		cv::cvtColor( input_image, input_image_gray, cv::COLOR_RGB2GRAY);

		cv::Mat input_image_smooth;
		if( gauss_smooth_sigma > 0.0f )
		{
			cv::GaussianBlur(input_image_gray,input_image_smooth,cv::Size(),gauss_smooth_sigma,gauss_smooth_sigma );
		}
		else
		{
			input_image_smooth=input_image_gray.clone();
		}
		cv::Mat gradient_x( input_image_smooth.rows, input_image_smooth.cols, CV_16S);
		cv::Mat gradient_y( input_image_smooth.rows, input_image_smooth.cols, CV_16S);
		cv::Sobel( input_image_smooth, gradient_x, CV_16S, 1, 0, sobel_winsize );
		cv::Sobel( input_image_smooth, gradient_y, CV_16S, 0, 1, sobel_winsize );

		/*cv::Mat aux;
		cv::resize( gradient_x, aux, cv::Size(),0.4,0.4 );
		cv::imshow( "gradent x", aux );
		cv::imshow( "gradent y", gradient_y );
		cv::waitKey(0);*/
		
		cv::Mat dbgimg;
		cv::cvtColor( input_image_smooth,dbgimg,cv::COLOR_GRAY2RGB);
		
		size_t num_refined = 0;
		for( size_t i=0; i<tags_found.size(); ++i )
		{
			for( cv::runetag::MarkerDetected::SlotIterator it = tags_found[i].begin(); it!=tags_found[i].end(); ++it )
			{
				if( it->getPayload() )
				{
					//cv::runetag::AuxRenderer::drawEllipsePoint( dbgimg, *(it->getPayload()), intrinsics, CV_RGB(255,0,0) );
					if( it->getPayload()->refine( gradient_x, gradient_y, intrinsics, dbgimg ) ) {
						num_refined++;
						//cv::runetag::AuxRenderer::drawEllipsePoint( dbgimg, *(it->getPayload()), intrinsics, CV_RGB(0,255,0) );
					}
				}
			}
		}

		/*		
		cv::imshow("refined",dbgimg);
		cv::waitKey(0);*/
		std::cout << "  " << num_refined << " refined. " << std::endl << std::endl;
	}


#ifdef ENABLE_TESTS
	if( tags_found.size() > 0 && doocclusiontest)
	{
		const cv::runetag::MarkerDetected& tag = tags_found[0];
		std::cout << "> Running occlusion test on the first Tag found (IDX " << tag.associatedModel()->getIDX() << ") " << std::endl;
		std::cout << "    # tests to perform: " << num_occlusion_tests << std::endl;
		std::cout << "    Interactive: " << (occlusion_test_interactive?"yes":"no") << std::endl;
		std::cout << "    result file: " << occlusion_test_filename << std::endl;

		RUNEtagdetect::test::occlusions::test_occlusions( tag, num_occlusion_tests, occlusion_test_interactive, occlusion_test_filename, input_image );

		std::cout << std::endl << "Test completed!" << std::endl;
	}
#endif

    std::cout << "> Estimating tags poses" << std::endl;
    cv::Mat distortion = cv::Mat::zeros(1,5,CV_32F);
    std::map< int, cv::runetag::Pose > poses;
    
    for( size_t i=0; i<tags_found.size(); ++i )
    {
        bool poseok;
        
        std::cout << "  Tag IDX:" << tags_found[i].associatedModel()->getIDX();

		unsigned int flags=0;
		if(pnpransac)
			flags |= cv::runetag::FLAG_REPROJ_ERROR;
		if(pnprefine)
			flags |= cv::runetag::FLAG_REFINE;

		cv::runetag::Pose RT = cv::runetag::findPose( 
                tags_found[i], intrinsics, distortion, &poseok, method==0?cv::SOLVEPNP_ITERATIVE:cv::SOLVEPNP_EPNP, flags);

        if( poseok )
        {
            poses[i]=RT;
            std::cout << " OK! R(rodriguez):  [" << RT.R.at<double>(0,0) << " ; " << RT.R.at<double>(1,0) << " ; " << RT.R.at<double>(2,0) << "]" << std::endl;        
            std::cout << "                     T:  [" << RT.t.at<double>(0,0) << " ; " << RT.t.at<double>(1,0) << " ; " << RT.t.at<double>(2,0) << "]" << std::endl;        
        } else
        {
            std::cout << " Error." << std::endl;
        }

    }
    std::cout << std::endl;



    if( tagsimg_filename.length()>0 )
    {
        cv::Mat dbgout = input_image.clone();
        std::cout << "> Rendering tags" << std::endl;
        std::cout << "   -> " << tagsimg_filename << std::endl << std::endl;

        for( size_t i=0; i<tags_found.size(); ++i )
        {
            if( poses.find(i)!=poses.end()) 
            {
                cv::runetag::AuxRenderer::drawDetectedMarker3DCylinder2(dbgout,tags_found[i],poses[i],intrinsics,distortion);
            }            
        }
        
        cv::imwrite( tagsimg_filename, dbgout );
    }


    if( points_filename.length()>0 )
    {
        std::cout << "> Saving tag points data" << std::endl;
        std::cout << "   -> " << points_filename << std::endl << std::endl;

        std::ofstream ofs( points_filename.c_str() );
        ofs.precision(15);
        ofs << std::scientific;
        if( !ofs.fail() )
        {
            for( size_t i=0; i<tags_found.size(); ++i )
            {
                for( size_t slot=0; slot<tags_found[i].getNumSlots(); ++slot )
                {
                    if( !tags_found[i].getSlot(slot).discarded() && tags_found[i].getSlot(slot).value() )
                    {
                        cv::Point2d point3d = cv::runetag::ellipseCenter( tags_found[i].associatedModel()->modelEllipseAtSlot(slot) );
                        cv::Point2d point2d = tags_found[i].getSlot(slot).getPayload()->getCenter() + cv::Point2d(intrinsics.at<double>(0,2),intrinsics.at<double>(1,2));

                        ofs << tags_found[i].associatedModel()->getIDX() << "\t" << slot << "\t" << point3d.x << "\t" << point3d.y << "\t" << "0" << "\t" << point2d.x << "\t" << point2d.y << std::endl;
                    }                    
                }
            }
            ofs.flush();
            ofs.close();


        } else 
        {
            std::cout << "Error: Unable to open file" << std::endl;

        }
    }


    if( pose_filename.length()>0 )
    {
        std::cout << "> Saving tag pose data" << std::endl;
        std::cout << "   -> " << pose_filename << std::endl << std::endl;

        std::ofstream ofs( pose_filename.c_str() );
        
        ofs.precision(15);
        ofs << std::scientific;
        if( !ofs.fail() )
        {
            for( std::map< int, cv::runetag::Pose >::const_iterator it = poses.begin(); it!=poses.end(); ++it )
            {
                const cv::runetag::Pose& pose = it->second;
                const cv::runetag::MarkerDetected& marker = tags_found[it->first];

                cv::Mat rrod;
                cv::Rodrigues( pose.R, rrod );
                double angle_deg = acos( rrod.at<double>(2,2) )*180/3.14;

                ofs << marker.associatedModel()->getIDX() << "\t" << angle_deg << "\t";

                for( int ii=0; ii<3; ++ii )
                    for( int jj=0; jj<3; ++jj )
                        ofs << rrod.at<double>(ii,jj) << "\t";

                ofs << pose.t.at<double>(0,0) << "\t" << pose.t.at<double>(1,0) << "\t" << pose.t.at<double>(2,0) << std::endl;
            }

            ofs.flush();
            ofs.close();

        } else 
        {
            std::cout << "Error: Unable to open file" << std::endl;

        }
    }

    

}
