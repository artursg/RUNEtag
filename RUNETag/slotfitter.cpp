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
#include "slotfitter.hpp"
#include "auxmath.hpp"
#include <iostream>
#include <fstream>

using namespace cv::runetag;

SlotFitter::SlotFitter( const cv::Matx33d& _VR, std::vector<EllipsePoint>& _ellipses, 
    const cv::Matx33d& fit_min, const cv::Matx33d& fit_max, const cv::Mat& _intrinsics, 
    int min_support, int min_symbols_for_layer ) : ellipses(_ellipses), 
    _min_symbols_for_layer(min_symbols_for_layer) 
{
    VR = cv::Matx33d(_VR);
    intrinsics = _intrinsics.clone();
    refit=cv::Matx33d();

    for( std::vector<EllipsePoint>::const_iterator it = ellipses.begin(); it<ellipses.end(); it++ ) 
    {
        const cv::Point2d center = it->getCenter();
        points_coords.push_back( center );
        const double x = center.x;
        const double y = center.y;
        const double& A_min = fit_min(0,0);
        const double& B_min = fit_min(0,1);
        const double& C_min = fit_min(1,1);
        const double& D_min = fit_min(0,2);
        const double& E_min = fit_min(1,2);
        const double& F_min = fit_min(2,2);

        if( A_min*x*x + 2.0*B_min*x*y + C_min*y*y + 2.0*D_min*x + 2.0*E_min*y + F_min > 0.0 ) 
        {
            const double& A_max = fit_max(0,0);
            const double& B_max = fit_max(0,1);
            const double& C_max = fit_max(1,1);
            const double& D_max = fit_max(0,2);
            const double& E_max = fit_max(1,2);
            const double& F_max = fit_max(2,2);

            if( A_max*x*x + 2.0*B_max*x*y + C_max*y*y + 2.0*D_max*x + 2.0*E_max*y + F_max < 0.0 ) 
            {
                EllipsePoint it_ell = *it;
                it_ell.toCircle( cv::Matx33d(VR) );
                auto_inliers.push_back( it_ell );
            }
        }
    }

    if( auto_inliers.size() < static_cast<unsigned int>(min_support) ) 
    {
        valid = false;
        return;
    }
    valid = true;
}

void SlotFitter::fit( std::vector< EllipsePoint >& ring_ellipses, double radius_ratio, double gap_factor, int num_layers, std::vector<MarkerDetected>& possible_markers ) 
{
    possible_markers.clear();
    for( unsigned int i=0; i<FIT_CACHE_SIZE; i++ ) 
    {
        fit_cache_exists[i]=false;
    }

    // Calc fitting circle in least squares sense
    const size_t N = ring_ellipses.size();
    double* x = new double[ N ];
    double* y = new double[ N ];
    double* u = new double[ N ];
    double* v = new double[ N ];

    double xavg = 0.0;
    double yavg = 0.0;
    for( unsigned int i=0; i<N; i++ ) 
    {
        cv::Point2d center = ring_ellipses[i].getCenterOnNormPlane();;
        x[i] = center.x;
        y[i] = center.y;
        xavg += x[i];
        yavg += y[i];
    }
    xavg /= (double)N;
    yavg /= (double)N;
    
    for( unsigned int i=0; i<N; i++ ) 
    {
        u[i] = x[i] - xavg;
        v[i] = y[i] - yavg;
    }

    double Suu=0.0;
    double Suv=0.0;
    double Svv=0.0;
    double Svvv=0.0;
    double Suuu=0.0;
    double Suvv=0.0;
    double Svuu=0.0;
    for( unsigned int i=0; i<N; i++ ) 
    {
        Suu  += u[i]*u[i];
        Suv  += u[i]*v[i];
        Svv  += v[i]*v[i];
        Svvv += v[i]*v[i]*v[i];
        Suuu += u[i]*u[i]*u[i];
        Suvv += u[i]*v[i]*v[i];
        Svuu += v[i]*u[i]*u[i];
    }

    delete[] x;
    delete[] y;
    delete[] u;
    delete[] v;

    cv::Mat A(2,2,CV_64F);
    cv::Mat b(2,1,CV_64F);

    A.at<double>(0,0) = Suu;
    A.at<double>(0,1) = Suv;
    A.at<double>(1,0) = Suv;
    A.at<double>(1,1) = Svv;

    b.at<double>(0,0) = 0.5*(Suuu+Suvv);
    b.at<double>(1,0) = 0.5*(Svvv+Svuu);

    cv::Mat ucvc(2,1,CV_64F);
    cv::solve( A, b, ucvc, cv::DECOMP_CHOLESKY );

    const double uc = ucvc.at<double>(0,0);
    const double vc = ucvc.at<double>(1,0);

    const float x_center = static_cast<float>(uc+xavg);
    const float y_center = static_cast<float>(vc+yavg);

    const double rsqr = uc*uc + vc*vc + (Suu+Svv)/N;
    const double r = sqrt(rsqr);

    refit(0,0) = 1.0;
    refit(1,1) = 1.0;
    refit(0,1) = 0.0;
    refit(1,0) = 0.0;
    refit(0,2) = refit(2,0) = -x_center;
    refit(1,2) = refit(2,1) = -y_center;
    refit(2,2) = refit(0,2)*refit(0,2) + refit(1,2)*refit(1,2) - rsqr;

    double f = (intrinsics.at<double>(0,0) + intrinsics.at<double>(1,1)) / 2.0;
    refined.ellipse_norm = transformToEllipse( refit, VR,  134.46);
    refined.ellipse = refined.ellipse_norm;
    refined.ellipse(0,2) *= f;
    refined.ellipse(1,2) *= f;
    refined.ellipse(2,0) *= f;
    refined.ellipse(2,1) *= f;
    refined.ellipse(2,2) *= f*f;

    //Create slots
    const double ellysize = 1.0 / radius_ratio;
    const cv::Point2d center( x_center, y_center );
    const cv::Point2d ce1 = ring_ellipses[0].getCenterOnNormPlane();
    double alpha = ellysize * 2.0 * gap_factor;
    const int num_slots_for_layer = static_cast<int>( floor( 6.2831853 / alpha ) );
    alpha = 6.2831853 / num_slots_for_layer;
    double e1_ang = atan2( ce1.y - center.y , ce1.x - center.x );
    if( e1_ang < 0.0 ) 
    {
        e1_ang += 6.2831853;
    }

    unsigned int num_wrong = 0;
    for( unsigned int i=1; i<N; i++ ) 
    {
        cv::Point2d ic = ring_ellipses[i].getCenterOnNormPlane();
        double ic_ang = atan2( ic.y - center.y , ic.x - center.x );
        if( ic_ang < 0.0 ) 
        {
            ic_ang += 6.2831853;
        }
        double ic_i = (ic_ang-e1_ang) / alpha;
        if( fabs( floor(ic_i+0.5)-ic_i ) > 0.20 ) 
        {
            num_wrong++;
        }
    }
    
    if( num_wrong > N/3 )
    {
        return;
    }

    //Transform from circle to ellipse world
    fit_slots.clear();
    fit_slots_centers.clear();

    cv::Point2d refit_center = ellipseCenter(refit);
    slots_center = transformPointFromCircleToEllipse( refit_center, VR, f );

    for( int i=0; i<num_slots_for_layer; i++ ) 
    {
        double angle = e1_ang + alpha * i + alpha*0.5;
        
        cv::Point2d curr_slot_p(  refit_center.x + r*cos(angle)*2.0 , refit_center.y + r*sin(angle)*2.0  );
        fit_slots.push_back( transformPointFromCircleToEllipse( curr_slot_p, VR, f ) );


        std::vector< cv::Point2f > level_slots;
        for( int j=0; j<num_layers; j++ ) 
        {
            const double factor = (  ((num_layers+j+1)*2.0)/(double)(num_layers*4.0)  );
            cv::Point2f curr_slot_centerp(  refit_center.x + r*factor*cos(angle-alpha*0.5) , refit_center.y + r*factor*sin(angle-alpha*0.5)  );
            level_slots.push_back( transformPointFromCircleToEllipse( curr_slot_centerp, VR, f ) );
        }        
        fit_slots_centers.push_back( level_slots );
    }

    for( int i=0; i<num_layers; i++ ) 
    {
        MarkerDetected newmarker = MarkerDetected::createDetectedMarker( num_layers );
        if( buildCodeForLayer( i, num_layers, rsqr, newmarker.code ) ) 
        {
            newmarker.VR = this->VR;
            possible_markers.push_back( newmarker );
        }
    }
}


void SlotFitter::fit( double radius_ratio, double gap_factor, int num_layers, std::vector<MarkerDetected>& possible_markers ) 
{
    fit( auto_inliers, radius_ratio, gap_factor, num_layers, possible_markers);
}


bool SlotFitter::buildCodeForLayer( int layer, int num_layers, double rSqr, std::vector< Slot >& code ) const 
{
    code.clear();
    const size_t fit_slots_size = fit_slots.size();
    code.resize( num_layers * fit_slots_size );
    const size_t code_size = code.size();
    const size_t points_coords_size = points_coords.size();

    for( size_t h=0; h<points_coords.size(); h++ )
    {
        ellipses[h].slotted = true;
    }

    std::vector< cv::Matx33d > fit_levels;
    int clayer = -layer;
    for( int j=0; j<=num_layers; j++ ) 
    {
        fit_levels.push_back(  getFitWithOffset( clayer-1, num_layers, rSqr ) );
        clayer+=1;
    }

    for( size_t f_level = fit_levels.size()-1; f_level>0; f_level-- ) 
    {
        cv::Matx33d qmax = fit_levels[f_level];
        cv::Matx33d qmin = fit_levels[f_level-1];

        bool this_layer_has_symbols = false;
        int num_symbols_for_this_layer = 0;

        for( size_t h=0; h<points_coords_size; h++ )
        {
            if( !ellipses[h].slotted || ellipses[h].isAssigned() )
            {
                continue;
            }

            const cv::Point2d& p = points_coords[h];
            if( ellipseContains( qmax, p ) ) 
            {
                //This ellipse is inside qmax
                if( !ellipseContains( qmin, p ) ) 
                {
                    //This ellipse is outside qmin and hence is a good candidate for this level
                    num_symbols_for_this_layer++;

                    for( unsigned int i=0; i<fit_slots_size; i++ ) 
                    {
                        int k = (i+1==fit_slots_size) ? 0 : i+1;

                        const cv::Point2d& c = slots_center;
                        const cv::Point2d& v1 = cv::Point2d( fit_slots[i].x - c.x, fit_slots[i].y - c.y);
                        const cv::Point2d& v2 = cv::Point2d( fit_slots[k].x - c.x, fit_slots[k].y - c.y);
                        Slot& curr_slot = code[i*num_layers + f_level - 1 ];
                        curr_slot.Qmax = qmax;
                        curr_slot.Qmin = qmin;
                        curr_slot.v1 = v1;
                        curr_slot.v2 = v2;
                        curr_slot.c = c;
                        curr_slot.slot_center = (fit_slots_centers[k])[f_level-1];

                        curr_slot.setIfInside(p, &(ellipses[h]));
                    }
                    ellipses[h].slotted = false; //outside qmin
                }
            } 
            else 
            {
                ellipses[h].slotted=false; //outside qmax
            }

        }

        if( num_symbols_for_this_layer <= _min_symbols_for_layer) 
        {
            return false;
        }

    }
    return true;
}


cv::Matx33d SlotFitter::getFitWithOffset( int currLayer, unsigned int num_layers, double radiusSqr ) const 
{
    if( fit_cache_exists[ currLayer+num_layers+1 ] ) 
    {
        return fit_cache[currLayer+num_layers+1];
    }

    const double f = (intrinsics.at<double>(0,0) + intrinsics.at<double>(1,1)) / 2.0;
    const double aux = refit(2,2);

    const double factor = (  ((num_layers+currLayer+1)*2.0+1)/(double)(num_layers*4.0)  );
    double rz0sqr = radiusSqr * factor * factor;
    
    refit(2,2) = -rz0sqr + refit(0,2)*refit(0,2) + refit(1,2)*refit(1,2);
    cv::Matx33d fit_off = transformToEllipse( refit, VR,  134.46);
    fit_off(0,2) *= f;
    fit_off(1,2) *= f;
    fit_off(2,0) *= f;
    fit_off(2,1) *= f;
    fit_off(2,2) *= f*f;

    refit(2,2) = aux;

    cv::Matx33d matrix(	fit_off(0,0), fit_off(0,1), fit_off(0,2),
                        fit_off(1,0), fit_off(1,1), fit_off(1,2),
                        fit_off(2,0), fit_off(2,1), fit_off(2,2));


    fit_cache_exists[ currLayer+num_layers+1 ] = true;
    fit_cache[currLayer+num_layers+1] = matrix;

    return matrix;
}

bool SlotFitter::ellipseContains( const cv::Matx33d& ellipse, const cv::Point2d& p ) 
{
    //Layer boundary
    const double& x = p.x;
    const double& y = p.y;
    const double& A_max = ellipse(0,0);
    const double& B_max = ellipse(0,1);
    const double& C_max = ellipse(1,1);
    const double& D_max = ellipse(0,2);
    const double& E_max = ellipse(1,2);
    const double& F_max = ellipse(2,2);

    return ( A_max*x*x + 2.0*B_max*x*y + C_max*y*y + 2.0*D_max*x + 2.0*E_max*y + F_max < 0.0 );
}

cv::Point2d SlotFitter::transformPointFromCircleToEllipse( cv::Point2d p, const cv::Matx33d& VR, double f ) 
{
    cv::Matx31d s_cn(p.x, p.y, 1.0);
    s_cn = VR*s_cn;

    return cv::Point2d( s_cn(0,0) / s_cn(2,0) * f , s_cn(1,0) / s_cn(2,0) * f );
}
