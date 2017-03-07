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

#include <iostream>
#include "runetag.hpp"
#include "coding.h"


void print_code( const std::vector<long>& code )
{
    for( size_t i=0; i<code.size(); ++i )
    {
        std::cout << code[i] << " ";
    }
    std::cout << std::endl;
}



int main (int argc, char** argv)
{
    std::map< long, std::vector<long> > generated_codes;

    cv::runetag::Coding coding;
    coding.init();

    long tidx=0;
    while( generated_codes.size()<17000 )
    {
        std::vector<long> code;
        try
        {
            long idx=coding.generate(code,tidx);
            if( coding.decode(code)==0 )
            {
                long rotation;
                long idxrot;
                coding.align( code, idxrot, rotation );
                if( idxrot==idx && rotation==0)
                {
                    generated_codes[idx] = code;
                    std::cout << generated_codes.size() << " codes generated" << std::endl;
                }                
            }
        }
        catch (std::invalid_argument& ex)
        {        	
            
        }
        tidx++;        
    }

    std::cout << "Saving data..." << std::endl;
    std::ofstream ofs( "codes.txt");
    ofs << generated_codes.size() << std::endl;

    for( std::map< long, std::vector<long> >::const_iterator it=generated_codes.begin(); it!=generated_codes.end(); ++it )
    {
        ofs << it->first << " " << it->second.size() << " ";
        std::vector<long> code = it->second;
        for( size_t i=0; i<code.size(); ++i )
        {
            ofs << code[i] << " ";
        }
        ofs << std::endl;
    }

    
    return 0;
}