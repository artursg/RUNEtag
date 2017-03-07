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
#include "digitalmarkermodel.hpp"
#include "markerdetected.hpp"

using namespace cv::runetag;

DigitalMarkerModel::DigitalMarkerModel() : radius_ratio(0.0), num_slots(0) {
    coding.init();
}

DigitalMarkerModel::DigitalMarkerModel(std::ifstream& ifs) 
{
    coding.init();
    std::string startString;
    ifs >> startString;
    if( startString.compare( std::string("RUNE_direct")) != 0 ) 
    {
        throw MarkerModelLoadException("Unable to read a MarkerModel descriptor.");
    }

    ifs >> name;

    ifs >> world_size;
    if( ifs.fail() ) 
    {
        throw MarkerModelLoadException("Unable to read world size.");
    }

    ifs >> world_measure_unit;
    if( ifs.fail() ) 
    {
        throw MarkerModelLoadException("Unable to read world measure unit.");
    }

    ifs >> num_slots;
    if( ifs.fail() ) 
    {
        throw MarkerModelLoadException("Unable to read number of slots.");
    }
    detected_errors.resize( num_slots );

    ifs >> radius_ratio;
    if( ifs.fail()  ) 
    {
        throw MarkerModelLoadException("Unable to read radius ratio.");
    }

    int num_layers;
    ifs >> num_layers;
    if( ifs.fail()  ) 
    {
        throw MarkerModelLoadException("Unable to read number of layers.");
    }
    if( num_layers != 3 )
    {
        throw MarkerModelLoadException("Invalid marker model, 3 layers expected");
    }

    ifs >> gap_factor;
    if( ifs.fail()  ) 
    {
        throw MarkerModelLoadException("Unable to read gap factor.");
    }

    ifs >> max_distance;
    if( ifs.fail()  ) 
    {
        throw MarkerModelLoadException("Unable to read max score.");
    }

    ifs >> idx;
    if( ifs.fail()  ) 
    {
        throw MarkerModelLoadException("Unable to read idx.");
    }

    // Load code
    for( unsigned int i=0; i<num_slots; i++ ) 
    {
        bool b;
        ifs >> b;
        if( ifs.fail() ) 
        {
            throw MarkerModelLoadException("Error while reading slot data");
        }
        bcode.push_back( b );
        if( b ) 
        {
            //read model ellipse
            cv::Matx33d me;
            for ( int j = 0; j < 3; j++ ) 
            {
                for ( int k = 0; k < 3; k++ ) 
                {
                    ifs >> me (j, k);
                }
            }
            me(1,2) *= -1.0;
            me(2,1) *= -1.0;

            if( ifs.fail() ) 
            {
                throw MarkerModelLoadException("Error while reading ellipse model data");
            }
            model_ellipses[i] = me;
        }
    }

    // Code sanity check
    code = coding.pack(bcode);

    std::vector< long > decoded = code;
    if( coding.decode( decoded )!= 0 )
    {
        throw MarkerModelLoadException("Invalid code. (Coder was unable to decode the given code)");
    }
    for( size_t i=0; i<decoded.size(); ++i )
    {
        if( decoded[i]!=code[i] )
            throw MarkerModelLoadException("Invalid code. (A correction is required)");
    }

    if( coding.get_index(code) != idx )
    {
        throw MarkerModelLoadException("Given and computed index mismatch.");
    }


    char buffer[64];
    ifs.getline( buffer, 64 ); //We expect a newline at the end of the descriptor
    ifs.getline( buffer, 64 );
}


DigitalMarkerModel::DigitalMarkerModel( const DigitalMarkerModel& other ) :
    name(other.name),
    radius_ratio(other.radius_ratio),
    gap_factor(other.gap_factor),
    num_slots(other.num_slots),
    max_distance(other.max_distance),
    max_symbols(other.max_symbols),
    bcode(other.bcode),
    code(other.code),
    idx(other.idx),
    model_ellipses(other.model_ellipses),
    world_size(other.world_size),
    world_measure_unit(other.world_measure_unit),
    detected_errors(other.detected_errors) 
    {
        coding.init();
    }



DigitalMarkerModel& DigitalMarkerModel::operator=( const DigitalMarkerModel& other ) 
{
    if( this == &other )
    {
        return *this;
    }

    name = other.name;
    radius_ratio = other.radius_ratio;
    gap_factor = other.gap_factor;
    num_slots = other.num_slots;
    max_symbols = other.max_symbols;
    max_distance = other.max_distance;
    bcode = other.bcode;
    code=other.code;
    idx=other.idx;
    model_ellipses = other.model_ellipses;
    world_size = other.world_size;
    world_measure_unit = other.world_measure_unit;
    detected_errors = other.detected_errors;

    return *this;
}


DigitalMarkerModel::~DigitalMarkerModel() 
{
    bcode.clear();
    code.clear();
}


bool DigitalMarkerModel::alignAndCheck( MarkerDetected & d ) const 
{
    throw MarkerModelOperationException("Direct RUNE tag must not be aligned like this");
    return false;
#if 0
    if( d.getNumSlots() != num_slots )
        return false;

    for( unsigned int offset=0; offset<num_slots; offset++ ) 
    {
        unsigned int align_score = 0;
        int num_errors = 0;
        for( unsigned int i=0; i<num_slots; i+=num_layers ) 
        {
            // Check if all symbols in this sector are missing and
            // and error has occurred
            bool all_missing = true;
            bool error_occurred = false;
            for( unsigned int j=0; j<num_layers; j++ ) 
            {
                if( d.getSlot(i+j).value() ) 
                {
                    all_missing = false;
                }
                if( d.getSlot(i+j).value() != code[i+j+offset] ) 
                {
                    error_occurred = true;
                    detected_errors[ num_errors++ ] = i+j;
                }
            }
            if( num_layers != 1 && all_missing ) 
            {
                align_score += 1;
                continue;
            }
            if( error_occurred ) 
            {
                align_score += 2;
                continue;
            }
        }

        if( align_score <= max_distance ) 
        {
            //remove errors so they do not affect further pose estimation
            for( int k=0; k<num_errors; k++) 
            {
                d.invalidateSlot(detected_errors[k]);
            }

            //A Valid match has been found
            d.associateModel( this, num_slots-offset );

            return true;
        }
    }
    return false;
#endif
}

bool DigitalMarkerModel::valueAtSlot( unsigned int slot_num ) const 
{
    return bcode[slot_num%num_slots];
}

