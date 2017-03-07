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
#ifndef RUNE_CODING_H
#define RUNE_CODING_H

#include<NTL/lzz_pE.h>
#include<NTL/lzz_pX.h>
#include<NTL/lzz_pE.h>
#include<NTL/lzz_pEX.h>

#include <vector>

namespace cv 
{
    namespace runetag
    {

        class Coding
        {
        public:
            Coding();
            void init();
            
            long get_index(const std::vector<long>& code_vec);
            void align(std::vector<long>& code_vec, long& index, long& rotation);
            int decode(std::vector<long>& code_vec);
            long generate(std::vector<long>& code, long index);

            static std::vector<bool> unpack( const std::vector<long> code );
            static std::vector<long> pack( const std::vector<bool> code );

        private:
            void Euclidean( NTL::zz_pEX& key, NTL::zz_pEX& locator, NTL::zz_pEX zeta,  long max_deg);
            void sft(std::vector<long>& out, const std::vector<long>& in);
            void isft(std::vector<long>& out, const std::vector<long>& in);

            const long start_range;
            const long end_range;
            const long code_length;
            const long num_words;
            std::vector<NTL::zz_pE> alpha;

            const long align_k;
            const long align_p;
            const long align_root;
            const long align_logn1;
            std::vector<long> align_pow;
            std::vector<long> align_log;

        };
        

    }
}

#endif