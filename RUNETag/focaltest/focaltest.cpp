#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <libGT/gt.h>
#include <iostream>
#include "runetag.hpp"
#include "auxrenderer.hpp"
#include "VirtualCamera.h"
#include "ellipsefitter.hpp"

#include "coding.h"

using namespace cv;
#define CREATE_SNAPSHOT

inline cv::Vec3d normalizevec( const cv::Vec3d vin )
{
    double len = sqrt(vin[0]*vin[0] + vin[1]*vin[1] + vin[2]*vin[2]);
    cv::Vec3d vout(vin);
    vout[0]/=len;
    vout[1]/=len;
    vout[2]/=len;
    return vout;
}


inline cv::Vec3d buildN( const cv::Matx33d& VR ) 
{
    //Ellipse normal transformed with VR is equal to VR*(0,0,1)'
    //that is equal to the column 2 of VR
    cv::Vec3d n;
    n[0] = VR(0,2);
    n[1] = VR(1,2);
    n[2] = VR(2,2);
    n = normalizevec(n);
    return n;
}


struct CandidatePlane
{
    CandidatePlane( const double _f, const cv::Vec3d _N, const cv::runetag::EllipsePoint _ellipse ) : f(_f), N(_N), ellipse(_ellipse) {}
    double f;
    cv::Vec3d N;
    cv::runetag::EllipsePoint ellipse;
};



class GTFocalAndPlaneInlierSelector
{
public:
    GTFocalAndPlaneInlierSelector( std::vector< cv::runetag::EllipsePoint >& _ellipses,
        double _focal_min,
        double _focal_max,
        int _num_steps ) : ellipses(_ellipses), focal_min(_focal_min), focal_max(_focal_max), nsteps(_num_steps)
    {

    }

    int generate_candidates()
    {
        candidates.clear();
        double focal_step = (focal_max-focal_min)/(double)nsteps;

        for( int i=0; i<ellipses.size(); ++i )
        {            
            for( double fx = focal_min; fx<=focal_max; fx+=focal_step )
            {
                ellipses[i].calcVR( fx );
                cv::Vec3d VR1_normal( buildN( ellipses[i].getVR1() ));
                cv::Vec3d VR2_normal( buildN( ellipses[i].getVR2() ));

                candidates.push_back( CandidatePlane(fx,VR1_normal,ellipses[i]));
                candidates.push_back( CandidatePlane(fx,VR2_normal,ellipses[i]));
            }
        }
        return (int)candidates.size();
    }
    

    void run()
    {
        int N = (int)candidates.size();
        population.resize( N );
        gt_create_population( &(population[0]), N);

        std::vector< double > A(N*N);

        // Build similarity matrix
        for( int i=0; i<N; ++i )
        {
            A[i*N+i] = 0.0;
            for( int j=i+1; j<N; ++j )
            {
                A[i*N+j] = A[j*N+i] = payoff( candidates[i], candidates[j] );
            }
        }

        double toll = 1e-10;
        int max_iters = 30000;
        gt_iidyn( &(A[0]), &(population[0]), N, &toll, &max_iters);
        std::cout << "GT compleded: " << toll << " errror, " << max_iters << " iters." << std::endl;
    }


    void get_inliers()
    {
        
        std::vector< double >::const_iterator maxel = std::max_element(population.begin(), population.end() );
        double maxval = *maxel;

        std::cout << "Best value: " << maxval << std::endl;

        double thresh = maxval*0.0;
        for( int i=0; i<population.size(); ++i )
        {
            if( population[i]>thresh )
            {
                std::cout << candidates[i].f << std::endl;

            }
        }
                
    }


    double payoff( const CandidatePlane& p1, const CandidatePlane& p2 )
    {
        if( p1.f != p2.f )
        {
            return 0;
        }
        return p1.N.ddot(p2.N);
    }

    std::vector< double > population;

private:    
    std::vector< CandidatePlane > candidates;

    std::vector< cv::runetag::EllipsePoint >& ellipses;
    double focal_min;
    double focal_max;
    int nsteps;
};



class FocalLengthEstimator
{
public:

    class PointNormal
    {
    public:
        double theta;   // Vector angle with respect to Z axis  [0..PI]
        double phi;     // Vector angle with respect to x axis (on x-y plane) [0..2PI)
        double f;       // Focal length
    };


    FocalLengthEstimator( std::vector< cv::runetag::EllipsePoint >& _ellipses,
                          double _focal_min,
                          double _focal_max,
                          int _num_steps ) : ellipses(_ellipses), focal_min(_focal_min), focal_max(_focal_max), nsteps(_num_steps)
    {

    }



    void compute_accumulator( std::string filename )
    {
        std::ofstream ofs( filename.c_str() );
        ofs << "theta\tphi\tf" << std::endl;


        double focal_step = (focal_max-focal_min)/(double)nsteps;

        for( int i=0; i<ellipses.size(); ++i )
        {            
            for( double fx = focal_min; fx<=focal_max; fx+=focal_step )
            {
                ellipses[i].calcVR( fx );
                cv::Vec3d VR1_normal( buildN( ellipses[i].getVR1() ));
                cv::Vec3d VR2_normal( buildN( ellipses[i].getVR2() ));

                PointNormal pn1 = vector2SphPnormal( VR1_normal, fx );
                PointNormal pn2 = vector2SphPnormal( VR2_normal, fx );

                ofs << pn1.theta << "\t" << pn1.phi << "\t" << pn1.f << "\t" << fx << std::endl;
                ofs << pn2.theta << "\t" << pn2.phi << "\t" << pn2.f << "\t" << fx << std::endl;
            }
        }

        ofs.close();

    }


    void compute_accumulator_xyz( std::string filename )
    {
        std::cout << "Generating normals..." << std::endl;
        std::ofstream ofs( filename.c_str() );
        ofs.precision(15);
        ofs << std::scientific;
        //ofs << "nx\tny\tnz" << std::endl;

        double focal_step = (focal_max-focal_min)/(double)nsteps;

        for( int i=0; i<ellipses.size(); ++i )
        {            
            for( double fx = focal_min; fx<=focal_max; fx+=focal_step )
            {
                ellipses[i].calcVR( fx );
                cv::Vec3d VR1_normal( buildN( ellipses[i].getVR1() ));
                cv::Vec3d VR2_normal( buildN( ellipses[i].getVR2() ));

                ofs << VR1_normal[0] << "\t" << VR1_normal[1] << "\t" << VR1_normal[2] << "\t" << fx << std::endl;
                ofs << VR2_normal[0] << "\t" << VR2_normal[1] << "\t" << VR2_normal[2] << "\t" << fx << std::endl;
            }
        }
        ofs.close();
        std::cout << "All done!" << std::endl;
    }



private:

    PointNormal vector2SphPnormal( const cv::Vec3d& v, double focal )
    {
        PointNormal pn;
        pn.theta = atan2( v[1], v[0] ) ;
        pn.phi = atan2( v[2], sqrt(v[0]*v[0] + v[1]*v[1]) );
        pn.f = focal;
        return pn;
    }

    cv::Vec3d buildNScale( const cv::Matx33d& VR, double focal ) 
    {
        //Ellipse normal transformed with VR is equal to VR*(0,0,1)'
        //that is equal to the column 2 of VR
        cv::Vec3d n;
        n[0] = VR(0,2)*focal;
        n[1] = VR(1,2)*focal;
        n[2] = VR(2,2)*focal;
        n = normalizevec(n);        
        return n;
    }

    std::vector< cv::runetag::EllipsePoint >& ellipses;
    double focal_min;
    double focal_max;
    int nsteps;
};



cv::Matx33d Q_old_way( const RotatedRect& rr, double f, double cx, double cy )
{
    cv::runetag::EllipsePoint e(rr,cx,cy );
    e.calcVR( f ); 
    return e.ellipse_norm;
}

cv::Matx33d Q_new_way( const RotatedRect& rr, double f, double cx, double cy )
{
    cv::runetag::EllipsePoint e(rr,0,0 );
    
    cv::Matx33d F = cv::Matx33d::eye();
    F(2,2)=1.0/f;

    cv::Matx33d C = cv::Matx33d::eye();
    C(0,2)=cx;
    C(1,2)=cy;
    
    return F*C.t()*e.ellipse*C*F;
}





//#define CREATE_SNAPSHOT
#if 1

int main (int argc, char** argv)
{

    cv::Mat frame;

    std::cout << "Enter f fmin fmax fsteps angle" << std::endl;
    double focal_len;
    double focal_min;
    double focal_max;
    double focal_steps;
    double angle;
    
        
    std::cin >> focal_len;
    std::cin >> focal_min;
    std::cin >> focal_max;
    std::cin >> focal_steps;
    std::cin >> angle;

    cv::Mat intrinsics = cv::Mat::eye(3,3,CV_64FC1);


#ifdef CREATE_SNAPSHOT
    {
        std::string img_file("virtualcamera/msingle.bmp");

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

    double cx = intrinsics.at<double>(0,2);
    double cy = intrinsics.at<double>(1,2);
    

    std::cout << "Using the follwing intrinsics matrix: " << std::endl << intrinsics << std::endl;

    std::vector< RotatedRect > foundEllipses;
    runetag::EllipseDetector ellipseDetector( 10, 500, 100.0f, 5000.0f, 0.3f, 0.30f, -0.5);    
    ellipseDetector.detectEllipses( frame, foundEllipses );

    std::cout << foundEllipses.size() << " ellipses found" << std::endl;


#if 0
    std::vector< cv::runetag::EllipsePoint > ellipses;
    for( int i=0; i<foundEllipses.size(); ++i )
    {
        ellipses.push_back( cv::runetag::EllipsePoint(foundEllipses[i],cx,cy) );
        ellipses.back().calcVR(f);
    }
    cv::runetag::EllipseFitter fitter( ellipses[1], ellipses[2], intrinsics, 0.97, 0.5 );      
    cv::Mat dbgframe = frame.clone();
    if( !fitter.fitEllipseAvg( 18.0 ) ) 
    {
        std::cout << "Fit ellipse failed" << std::endl;
        return 0;
    }
    {
        cv::Matx33d big_ellipse = fitter.getFit1WithOffset(0.0);
        cv::runetag::AuxRenderer::drawEllipse( dbgframe, (cv::Mat)big_ellipse, intrinsics, CV_RGB(255,0,0));
    }
    {
        cv::Matx33d big_ellipse = fitter.getFit2WithOffset(0.0);
        cv::runetag::AuxRenderer::drawEllipse( dbgframe, (cv::Mat)big_ellipse, intrinsics, CV_RGB(0,255,0));
    }
    cv::Matx33d Rbase = fitter.VR;

    cv::imshow( "dbg", dbgframe );
    cv::waitKey(0);


    // Export data
    {
        std::ofstream ofs( "camera_parameters_gt.txt");
        ofs.precision(15);
        ofs << "focal_length cx cy" << std::endl;
        ofs << f << " " << cx << " " << cy << std::endl;
        ofs.close();
    }
    {
        std::ofstream ofs( "rbase.txt");
        ofs.precision(15);
        ofs << Rbase(0,0) << " " << Rbase(0,1) << " " << Rbase(0,2) << std::endl;
        ofs << Rbase(1,0) << " " << Rbase(1,1) << " " << Rbase(1,2) << std::endl;
        ofs << Rbase(2,0) << " " << Rbase(2,1) << " " << Rbase(2,2) << std::endl;
        ofs.close();
    }

    // Export ellipses
    std::ofstream ofs_e("ellipses.txt");
    ofs_e.precision(15);
    ofs_e << std::scientific;
    std::ofstream ofs_echeck("ellipses_check.txt");
    ofs_echeck.precision(15);
    ofs_echeck << std::scientific;
    for( int i=0; i<foundEllipses.size(); ++i )
    {
        cv::runetag::EllipsePoint e(foundEllipses[i],0,0);        
        ofs_e << e.ellipse(0,0) << " " << e.ellipse(0,1) << " " << e.ellipse(0,2) << std::endl;
        ofs_e << e.ellipse(1,0) << " " << e.ellipse(1,1) << " " << e.ellipse(1,2) << std::endl;
        ofs_e << e.ellipse(2,0) << " " << e.ellipse(2,1) << " " << e.ellipse(2,2) << std::endl;

        cv::runetag::EllipsePoint eC(foundEllipses[i],cx,cy);
        eC.calcVR( f ); 
        ofs_echeck << eC.ellipse_norm(0,0) << " " << eC.ellipse_norm(0,1) << " " << eC.ellipse_norm(0,2) << std::endl;
        ofs_echeck << eC.ellipse_norm(1,0) << " " << eC.ellipse_norm(1,1) << " " << eC.ellipse_norm(1,2) << std::endl;
        ofs_echeck << eC.ellipse_norm(2,0) << " " << eC.ellipse_norm(2,1) << " " << eC.ellipse_norm(2,2) << std::endl;
    }
    ofs_e.close();
    ofs_echeck.close();

    // Export Rbase
#endif


#if 0
    std::vector< cv::runetag::EllipsePoint > ellipses;
    for( int i=0; i<foundEllipses.size(); ++i )
    {
        ellipses.push_back( cv::runetag::EllipsePoint(foundEllipses[i],cx,cy) );
    }
    
    GTFocalAndPlaneInlierSelector inl(ellipses, 900, 1100, 200 );
    int n_candidates = inl.generate_candidates();
    std::cout << n_candidates << " candidates generated" << std::endl;
    inl.run();
    inl.get_inliers();

    // Save population
    {
        std::ofstream ofs("population.txt");
        for( int i=0; i<inl.population.size(); ++i )
        {
            ofs << inl.population[i] << std::endl;
        }
        ofs.close();
    }
    


#endif

#if 0

    std::vector< cv::runetag::EllipsePoint > ellipses;
    for( int i=0; i<foundEllipses.size(); ++i )
    {
        ellipses.push_back( cv::runetag::EllipsePoint(foundEllipses[i],cx,cy) );
    }

    FocalLengthEstimator fle( ellipses, 500, 2000, 500 );
    //fle.compute_accumulator("data_angle_test.txt");
    fle.compute_accumulator_xyz( "data_xyz_test.txt" );


#endif

#if 1
    
    std::vector< cv::runetag::EllipsePoint > ellipses;
    cv::Mat dbgframe = frame.clone();
    for( int i=0; i<foundEllipses.size(); ++i )
    {
        ellipses.push_back( cv::runetag::EllipsePoint(foundEllipses[i],cx,cy) );
        cv::runetag::AuxRenderer::drawEllipsePoint( dbgframe, ellipses.back(), intrinsics, CV_RGB(0,255,0));
    }
    cv::imwrite("ellipses.png", dbgframe );

    FocalLengthEstimator fle( ellipses, focal_min, focal_max, focal_steps );
    fle.compute_accumulator_xyz("data_fmin_" +cv::runetag::AuxRenderer::toStr(focal_min) + 
                                    "_fmax_" +cv::runetag::AuxRenderer::toStr(focal_max) +  
                                    "_fstp_" +cv::runetag::AuxRenderer::toStr(focal_steps) +  
                                    "_f_" +cv::runetag::AuxRenderer::toStr(focal_len) +  
                                    "_angle_"+cv::runetag::AuxRenderer::toStr(angle)+".txt");    
    
#endif

#if 0
    for( int i=0; i<foundEllipses.size(); ++i )
    {
        for( int j=i+1; j<foundEllipses.size(); ++j )
        {
            cv::Mat dbgframe = frame.clone();
            
            std::vector< cv::runetag::EllipsePoint > ellipses;
            ellipses.push_back( cv::runetag::EllipsePoint(foundEllipses[i],cx,cy) );
            ellipses.push_back( cv::runetag::EllipsePoint(foundEllipses[j],cx,cy) );

            float fx=1000.0f;
            //for( float fx=300.0f; fx<1800.0f; fx+=100 )
            {
                std::cout << i << "-" << j << std::endl;
                intrinsics.at<double>(0,0)=intrinsics.at<double>(1,1)=fx;               

                for( size_t ii=0; ii<2; ++ii )
                {
                    ellipses[ii].calcVR( fx );        
                    cv::runetag::AuxRenderer::drawEllipsePoint( dbgframe, ellipses[ii], intrinsics, CV_RGB(0,0,255) );
                }

                cv::runetag::EllipseFitter fitter( ellipses[0], ellipses[1], intrinsics, 0.97,0.5 );      
                if( !fitter.fitEllipseAvg( 18.0 ) ) 
                {
                    std::cout << "Fit ellipse from pair " << i << " " << j << " failed" << std::endl;
                    continue;
                }
                {
                    cv::Matx33d big_ellipse = fitter.getFit1WithOffset(0.0);
                    cv::runetag::AuxRenderer::drawEllipse( dbgframe, (cv::Mat)big_ellipse, intrinsics, CV_RGB(255,0,0));
                }
                {
                    cv::Matx33d big_ellipse = fitter.getFit2WithOffset(0.0);
                    cv::runetag::AuxRenderer::drawEllipse( dbgframe, (cv::Mat)big_ellipse, intrinsics, CV_RGB(0,255,0));
                }

            }


            cv::imshow( "dbg", dbgframe );
            cv::waitKey(0);
        }
    }
    
#endif

    

    std::cout << "Press ENTER to exit" << std::endl;
    return 0;
}
#endif