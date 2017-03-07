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
#include "markerdetector.hpp"
#include "ellipsefitter.hpp"
#include "slotfitter.hpp"
#include "auxrenderer.hpp"
#include <fstream>
#include <iostream>

using namespace cv::runetag;


/************************************************************************/
/*   Utility functions                                                  */
/************************************************************************/

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


void show_detected_marker( const cv::Mat& dbg_img, const cv::runetag::MarkerDetected& marker, const cv::Mat& intrinsics )
{
    std::cout << "Marker detected: IDX " << marker.associatedModel()->getIDX() << " errors: " << marker.num_errors << " discarded: " << marker.num_discarded << std::endl;
    
    cv::Mat dbg = dbg_img.clone();
    AuxRenderer::drawDetectedMarker( dbg, marker, intrinsics );
    cv::imshow("dbg",dbg);
    while( cv::waitKey(0)!='a' );
    
}


/************************************************************************/
/*   Class methods                                                      */
/************************************************************************/

MarkerDetector::MarkerDetector( const cv::Mat& _intrinsics ) 
{
    intrinsics = _intrinsics.clone();
    min_pts_for_level = 4;
}

MarkerDetector::~MarkerDetector() {}

int MarkerDetector::addModelsFromFile( std::string filename ) 
{
    int num_loaded = 0;

    std::ifstream ifs( filename.c_str() );
    if( !ifs.is_open() ) 
    {
        return num_loaded;
    }

    do 
    {
        DigitalMarkerModel newmodel(ifs);

        // Check if the new loaded markers are compatible with older ones
        if( !models.empty() ) 
        {
            if( !models.begin()->second.compatible( newmodel) ) 
            {
                throw MarkerDetector::MarkerDetectorException("Trying to load a MarkerModel witch is not compatible with the least recently loaded.");
            }
        }

        // Add the model
        models[newmodel.getIDX()] = newmodel;
        num_loaded++;
    } 
    while( !ifs.eof() );
    
    return num_loaded;
}


void MarkerDetector::toEllipsePoints( const std::vector<cv::RotatedRect>& ellipses ) 
{
    for( std::vector<cv::RotatedRect>::const_iterator it = ellipses.begin(); it!=ellipses.end(); it++ ) 
    {
        cv::RotatedRect r = *it;
        EllipsePoint newep( r, intrinsics.at<double>(0,2), intrinsics.at<double>(1,2) );
        newep.calcVR( (intrinsics.at<double>(0,0)+intrinsics.at<double>(1,1))/2.0 );
        ellipsePoints.push_back( newep );
    }
}


bool MarkerDetector::tryFit( SlotFitter& fitter, std::map<int, MarkerDetected>& markers_by_id ) 
{
    std::vector<MarkerDetected> possible_markers;
    std::vector<MarkerDetected> found_markers;

    fitter.fit( models.begin()->second.getRadiusRatio(), 
                models.begin()->second.getGapFactor(), 
                models.begin()->second.getNumLayers(), 
                possible_markers );

    

    if( findModel(possible_markers, found_markers) )
    {
#if 0 
        // A marker was found. so we can try to refine it
        //std::cout << "Marker detected: IDX " << found_markers[0].associatedModel()->getIDX() << " errors: " << found_markers[0].num_errors << " discarded: " << found_markers[0].num_discarded << std::endl;
        show_detected_marker( dbgimage, found_markers[0], intrinsics );
        std::cout << "Refining..." << std::endl;

        unsigned int fullest_layer = found_markers[0].getFullestLayer();

        std::vector< cv::Point2d > refit_pts;

        std::vector<EllipsePoint> fullest_layer_ellipses;
        for( MarkerDetected::SlotIterator it=found_markers[0].begin_by_layer(fullest_layer); it!=found_markers[0].end_by_layer(fullest_layer); ++it )
        {
            if( it->value() )
            {
                EllipsePoint ep = *(it->getPayload());
                refit_pts.push_back( ep.getCenter() );
            }
        }

        cv::RotatedRect refit_ellipse = cv::fitEllipse( refit_pts );
        

        
        // Modify fitters inliers to hopefully obtain a better estimation
        std::vector<EllipsePoint> fullest_layer_ellipses;
        for( MarkerDetected::SlotIterator it=found_markers[0].begin_by_layer(fullest_layer); it!=found_markers[0].end_by_layer(fullest_layer); ++it )
        {
            if( it->value() )
            {
                EllipsePoint ep = *(it->getPayload());
                ep.toCircle( found_markers[0].getVR() );
                fullest_layer_ellipses.push_back( ep );
            }
        }
#endif
        
#if 0

        possible_markers.clear();
        found_markers.clear();
        fitter.fit( fullest_layer_ellipses,
            models.begin()->second.getRadiusRatio(), 
            models.begin()->second.getGapFactor(), 
            models.begin()->second.getNumLayers(), 
            possible_markers );

        if( findModel(possible_markers, found_markers) )
        {
            std::cout << "refined FIT" << std::endl;
            show_detected_marker( dbgimage, found_markers[0], intrinsics );
            return true;
        }
#endif

        // Set detected marker ellipses as "assigned" to speed up computation of the subsequent markers
        /*for( unsigned int i=0; i<found_markers[0].getNumSlots(); ++i )
        {
            if( found_markers[0].getSlot(i).value() )
            {
                found_markers[0].getSlot(i).getPayload()->setAssigned();
            }
        }*/
        long marker_idx = found_markers[0].associatedModel()->getIDX();
        if( markers_by_id.find( marker_idx ) !=  markers_by_id.end() )
        {
            // This marker was already found, so we overwrite the old one if this one was better recognized
            if( found_markers[0].getNumFilledSlots() > markers_by_id[marker_idx].getNumFilledSlots() )
                markers_by_id[marker_idx] = found_markers[0];
        }
        else
        {
            markers_by_id[marker_idx] = found_markers[0];
        }
        
        return true;
        
    }
    return false;
}





int MarkerDetector::detectMarkers( const std::vector<cv::RotatedRect>& ellipses, std::vector<MarkerDetected>& markers_detected ) 
{
    std::map<int, MarkerDetected> markers_by_id;

    ellipsePoints.clear();

    // If models are empty there's nothing to detect
    if( models.empty() ) 
    {
        std::cout << "Model is Empty!" << std::endl;
        return 0;
    }

    // Creating an EllipsePoint for each ellipse detected
    toEllipsePoints( ellipses );

    // Creating all pairs of ellipsePoints
    for( unsigned int i=0; i<ellipsePoints.size(); i++ ) 
    {
        for( unsigned int j=i+1; j<ellipsePoints.size(); j++ ) 
        {
            //std::cout << i << " - " << j << std::endl;
            EllipsePoint& e1 = ellipsePoints[i];
            EllipsePoint& e2 = ellipsePoints[j];

            if( e1.isAssigned() || e2.isAssigned() ) 
            {
                //std::cout << "Skipping (already assigned)" << std::endl;
                continue;
            }

            EllipseFitter fitter( e1, e2, intrinsics );
            if( !fitter.fitEllipseAvg( models.begin()->second.getRadiusRatio() ) ) 
            {
                // Updating statistics, pairs of ellipses not fitted
                continue;
            }

            SlotFitter sf_fit1( fitter.getVR(), ellipsePoints, fitter.getFit1WithOffset(-2.3), fitter.getFit1WithOffset(2.3), intrinsics, min_pts_for_level );
            if( !sf_fit1.isValid() ) 
            {
                continue;
            }
            if ( tryFit( sf_fit1, markers_by_id ) ) 
            {
                continue;
            }
    
            SlotFitter sf_fit2( fitter.getVR(), ellipsePoints, fitter.getFit2WithOffset(-2.3), fitter.getFit2WithOffset(2.3), intrinsics, min_pts_for_level );
            if( !sf_fit2.isValid() ) 
            {
                continue;
            }
            if ( tryFit( sf_fit2, markers_by_id ) ) 
            {
                continue;
            }
        }
    }

    for( std::map<int, MarkerDetected>::const_iterator it= markers_by_id.begin(); it!=markers_by_id.end(); ++it )
        markers_detected.push_back( it->second );

    return (int)markers_detected.size();
}



int MarkerDetector::detectMarkers( const std::vector<cv::RotatedRect>& ellipses, std::vector<MarkerDetected>& markers_detected, DetectorStats& stats ) 
{
    return 0;
}



bool MarkerDetector::findModel( const std::vector<MarkerDetected>& possible_markers, std::vector<MarkerDetected>& markers_detected ) 
{
    cv::runetag::Coding coding;
    coding.init();

    bool found = false;
    for( unsigned int layer_guess=0; layer_guess<possible_markers.size(); layer_guess++ ) 
    {
        MarkerDetected curr_det = possible_markers[layer_guess];        
                
        std::vector< bool > bcode( curr_det.getNumSlots() );
        for( size_t i=0; i<curr_det.getNumSlots(); ++i )
        {
            bcode[i]=curr_det.getSlot(i).value();
        }

        try 
        {
            std::vector< long > detected_code = coding.pack( bcode );
            //std::cout << "Binary code: " << std::endl;
            //print_code(std::cout, bcode);
            //std::cout << "Code: " << std::endl;
            //print_code(std::cout, detected_code);

            if( coding.decode( detected_code ) == 0 )
            {
                // Decoding was successful
                //std::cout << "Decoded: " << std::endl;
                //print_code(std::cout, detected_code);
                                
                long idx_detected;
                long rotation;
                std::vector<long> aligned_code;
                coding.align( detected_code, idx_detected, rotation );
                
                if( models.find(idx_detected) != models.end() )
                {
                    // We know this model!                    
                    //std::cout << "idx " << idx_detected << std::endl;
                    //std::cout << "rotation " << rotation << std::endl;

                    // Apply rotation
                    curr_det.associateModel( &(models[idx_detected]), curr_det.offset + rotation*3);
                    
                    // Remove errors (this is needed to avoid problems later on with pose estimation)
                    bcode = coding.unpack( detected_code );
                    for( size_t i=0; i<curr_det.getNumSlots(); ++i )
                    {
                        if( curr_det.getSlot(i).value()!=bcode[i] )
                        {
                            curr_det.num_errors++;
                            curr_det.invalidateSlot(i);

                            if( curr_det.getSlot(i).value() )
                                curr_det.num_discarded++;
                        }
                    }

                    markers_detected.push_back( curr_det );
                    return true;
                }
            }

        } catch( std::invalid_argument ex )
        {

        }
    }
    return false;
}
