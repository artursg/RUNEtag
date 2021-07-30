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


#ifndef DIGITAL_MARKER_MODEL_H
#define DIGITAL_MARKER_MODEL_H

//#include "precomp.hpp"
#include <opencv2/core/core.hpp>

#include <fstream>
#include <string>
#include <map>
#include <iostream>

#include "coding.h"

namespace cv 
{

namespace runetag
{

class MarkerDetected;

class DigitalMarkerModel 
{

private:

    std::string name;
    std::string world_measure_unit;
    unsigned int num_slots;
    unsigned int max_distance;
    unsigned int max_symbols;
    double radius_ratio;
    double gap_factor;
    double world_size;

    std::vector<bool> bcode;
    std::vector<long> code;
    long idx;

    cv::runetag::Coding coding;

    mutable std::map<int, cv::Matx<double,3,3> > model_ellipses;
    mutable std::vector<unsigned int> detected_errors;

public:

    DigitalMarkerModel();

    DigitalMarkerModel(std::ifstream& ifs);

    DigitalMarkerModel( const DigitalMarkerModel& other );

    DigitalMarkerModel& operator=( const DigitalMarkerModel& other );

    ~DigitalMarkerModel();

    ///<summary>Checks if a detected marker can be matched with this model, and aligns it properly if possible.</summary>
    ///<param name="d">Detected marker to be tested</param>
    ///<returns>True if a valid alignment has been found, False otherwise</returns>
    bool alignAndCheck( MarkerDetected & d ) const;

    bool valueAtSlot( unsigned int slot_num ) const;

    ///<summary>Outputs a description of a MarkerModel to the standard output</summary>
    ///<param name="os">Output stream</param>
    ///<param name="m">Marker model</param>
    friend std::ostream& operator<<(std::ostream& os, const DigitalMarkerModel& m)
    {
        os << "RUNE-Tag_direct " << m.name << "[IDX=" << m.getIDX() <<  "]" << m.num_slots << " slots, " << m.max_distance 
            << " max distance)" << std::endl;
        return os;
    }


    inline bool compatible( const DigitalMarkerModel& other )
    {
        return (num_slots == other.num_slots && radius_ratio == other.radius_ratio && gap_factor == other.gap_factor );
    }

    inline const cv::Matx<double,3,3>& modelEllipseAtSlot( unsigned int slot_num ) const
    {
        if( bcode[slot_num] )
        {
            return model_ellipses[slot_num];
        }
        throw MarkerModelOperationException("No model ellipse exists at this slot");
    }

    inline unsigned int numSymbols() const
    {
        unsigned int count=0;
        for( unsigned int i=0; i<bcode.size(); i++ ) 
        {
            if( bcode[i] )
            {
                count++;
            }
        }
        return count;
    }

    inline bool isInternal( unsigned int slot_num ) const 
    {
        return (slot_num%getNumLayers() == 0 );
    }

    inline unsigned int numSymbolsInternal() const 
    {
        unsigned int count=0;
        for( unsigned int i=0; i<bcode.size(); i++ )
        {
            if( isInternal(i) && bcode[i] )
            {
                count++;
            }
        }
        return count;
    }

    inline std::string getName() const 
    { 
        return name; 
    }

    inline long getIDX() const
    {
        return idx;
    }
    
    inline double getGapFactor() const 
    { 
        return gap_factor; 
    }
    
    inline double getRadiusRatio() const 
    { 
        return radius_ratio; 
    }
    
    inline unsigned int getNumLayers() const 
    { 
        return 3; 
    }
    
    inline unsigned int getNumSlots() const
    {
        return num_slots;
    }
    
    inline double getWorldSize() const
    {
        return world_size;
    }

    class MarkerModelLoadException : public std::runtime_error 
    {
    public:
        MarkerModelLoadException(const std::string& reason ) : std::runtime_error( reason ) {};
    };

    class MarkerModelOperationException : public std::runtime_error 
    {
    public:
        MarkerModelOperationException(const std::string& reason ) : std::runtime_error( reason ) {};
    };
};

} // namespace runetag
} // namespace cv

#endif
