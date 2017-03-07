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

#include "coding.h"
#include<iostream>
#include<algorithm>
#include<stdexcept>
#include<cstdlib>
//#include<sys/time.h>



using cv::runetag::Coding;


std::vector<bool> Coding::unpack( const std::vector<long> code )
{
    std::vector<bool> bitcode( code.size()*3 );

    size_t idx=0;
    for( std::vector<long>::const_iterator it=code.begin(); it!=code.end(); ++it )
    {
        const long& c = *it + 1;    // unpacked code should go from 1 to 7 as 0 is reserved to erasures
        if( c > 7 || c < 1 )
            throw std::invalid_argument("Invalid code");


        bitcode[idx++] = ((c/4)%2 );
        bitcode[idx++] = ((c/2)%2 );
        bitcode[idx++] = ((c)%2 );
    }
    return bitcode;
}

std::vector<long> Coding::pack( const std::vector<bool> bitcode )
{
    if (bitcode.size()%3!=0) throw std::invalid_argument("Wrong code length");
    std::vector<long> code( bitcode.size()/3 );

    size_t idx=0;
    size_t cidx=0;
    while( idx<bitcode.size() )
    {
        code[cidx] = (long)(bitcode[idx])*4 + (long)(bitcode[idx+1])*2 + (long)(bitcode[idx+2]);
        code[cidx]--;

        idx+=3;
        cidx++;
    }

    return code;
}


// Computer the i-th syndrome of code
// ai is the i-th power of the generating root of unity a
inline void Syndrome(NTL::zz_pE& S, const NTL::zz_pX code, const NTL::zz_pE ai)
{ NTL::CompMod(S.LoopHole(), code, NTL::_zz_pE__rep(ai), NTL::zz_pE::modulus()); }


static long generator[43] = {1, 1, 6, 4, 6, 0, 3, 1, 5, 3, 5, 4, 0, 4, 6, 3, 4, 6, 3, 6, 4, 3, 6, 4, 0, 4, 5, 3, 5, 1, 3, 0, 6, 4, 6, 1, 1, 0, 0, 0, 0, 0, 0};


Coding::Coding() : start_range(8), end_range(36), code_length(43), num_words(117649), align_k(4), align_p(173), align_root(2), align_logn1(89)
{

}



void coding_error_callback(void)
{
    throw std::invalid_argument("NTL error");
}


void Coding::init()
{
    //initialize finite field Z_7^6
    NTL::zz_p::init(7);
    NTL::zz_pX P = NTL::zz_pX(6,1)+NTL::zz_pX(5,6)+NTL::zz_pX(3,2)+NTL::zz_pX(1,6)+1L; //P irreducible polynomial
    NTL::zz_pE::init(P);


    // root of unity generating the generalized BCH code
    NTL::zz_pX wx = NTL::zz_pX(5,1)+NTL::zz_pX(4,4)+NTL::zz_pX(2,5)+NTL::zz_pX(1,6);
    NTL::zz_pE w,wi;//=power(w,start_range);
    NTL::conv(w,wx);
    NTL::set(wi);
    alpha.resize(code_length+1);
    for (long i=0; i!=code_length+1; ++i)
    {
        alpha[i]=wi;
        wi *= w;
    }

    align_pow.resize(align_p);
    align_log.resize(align_p);
    long a=1L;
    for(long i=0; i!=align_p-1; ++i)
    {
        align_pow[i]=a;
        align_log[a]=i;
        a = (a*align_root)%align_p;
    }

    NTL::ErrorCallback = coding_error_callback;
}





void Coding::Euclidean( NTL::zz_pEX& key, NTL::zz_pEX& locator, NTL::zz_pEX zeta,  long max_deg)
{
    NTL::zz_pEX U(0,1L), V(0,0L), K, M, N, Q, R;
    key = NTL::zz_pEX(end_range-start_range,1L);
    while (NTL::deg(key) > max_deg)
    {
        NTL::DivRem(Q,R,key,zeta);
        M = locator - U*Q;
        N = K - V*Q;
        key = zeta;
        zeta = R;
        locator = U;
        K = V;
        U = M;
        V = N;
    }
}



void Coding::sft(std::vector<long>& out, const std::vector<long>& in)
{
    for (long i=0; i!=code_length; ++i)
    {
        out[i]=0;
        const long strobe = (align_k*i)%(align_p-1);
        for (long j=0; j!=code_length; ++j)
        {
         out[i] += align_pow[(strobe*j)%(align_p-1)]*in[j];
         out[i]=out[i]%align_p;
         //std::cerr << align_pow[(strobe*j)%(align_p-1)] << " ";
        }
        //std::cerr << "\n";
    }
    //std::cerr << "\n";
}

void Coding::isft(std::vector<long>& out, const std::vector<long>& in)
{
    for (long i=0; i!=code_length; ++i)
    {
        out[i]=0;
        const long strobe = (align_k*i)%(align_p-1);
        for (long j=0; j!=code_length; ++j)
        {
            const long psn = (strobe*j)%(align_p-1);
            out[i] += align_pow[( align_logn1+align_p-2-psn )%(align_p-1)]*in[j];
            out[i]=out[i]%align_p;
            //std::cerr << align_pow[( align_logn1+align_p-2-psn )%(align_p-1)] << " ";
        }
        //std::cerr << "\n";
    }
    //std::cerr << "\n";
}


long Coding::get_index(const std::vector<long>& code_vec)
{
    long index=0;
    index += (5*code_vec[0]+2*code_vec[3]+6*code_vec[4]+code_vec[5])%7;
    index *=7;
    index += (2*code_vec[2]+6*code_vec[3]+code_vec[4])%7;
    index *=7;
    index += (2*code_vec[1]+6*code_vec[2]+code_vec[3])%7;
    index *=7;
    index += (2*code_vec[0]+6*code_vec[1]+code_vec[2])%7;
    index *=7;
    index += (6*code_vec[0]+code_vec[1])%7;
    index *=7;
    index += code_vec[0];

    return index;
}


void Coding::align(std::vector<long>& code_vec, long& index, long& rotation)
{
    std::vector<long> FT(code_length);
    sft(FT,code_vec);


    if (FT[1]==0) throw std::invalid_argument("periodic code");


    rotation=align_log[FT[1]]/align_k;
    const long rot_idx=align_p-1-align_k*rotation;
    for (long i=1; i!=code_length; ++i)
        FT[i] = (FT[i]*align_pow[(rot_idx*i)%(align_p-1)])%align_p;

    isft(code_vec,FT);
    index=get_index(code_vec);

}


int Coding::decode(std::vector<long>& code_vec)
{
    NTL::zz_pX code;                //received code
    code.rep.SetLength(code_length);
    NTL::zz_pEX Ex(0,1L);           //erasure polynomial
    long erasure_num=0;
    for (long i=0; i!=std::min(code_length,static_cast<long>(code_vec.size())); ++i)
    {
        if (code_vec[i]>=0 && code_vec[i]<7)
            code.rep[i] = code_vec[i];
        else
        {
            code.rep[i] = 0L;
            ++erasure_num;
            Ex *= NTL::zz_pEX(1,-alpha[i])+1;
        }
    }
    code.normalize();
    const long t=(end_range-start_range-erasure_num)/2;

    NTL::zz_pEX Sx;                 //syndrome polynomial
    Sx.rep.SetLength(end_range-start_range);
    for (long i=0; i!=end_range-start_range; ++i)
    {
        Syndrome(Sx.rep[i], code, alpha[start_range+i]);
    }
    Sx.normalize();
    code.rep.SetLength(code_length);

    if (!NTL::IsZero(Sx))
    {
        NTL::zz_pEX O, Lx, DLx;

        //Euclidean algorithm
        Euclidean(O, Lx, NTL::MulTrunc(Ex,Sx,end_range-start_range), t+erasure_num);
        // O: key (error-evaluator) polynomial
        // Lx: error locator polynomial

        //std::cerr << Lx << "\n" << Ex << "\n";
        //Forney's algorithm
        Lx *= Ex;
        NTL::diff(DLx,Lx);
        // Lx: error+erasure locator polynomial
        // DLx: symbolic derivative of Lx

        //locate errors
        for (long pos=0; pos !=code_length; ++pos)
        {
            if (NTL::IsZero(eval(Lx, alpha[code_length-pos])))
            {
                // error or erasure
                NTL::zz_pE e = alpha[code_length-(pos*(start_range-1))%code_length] *
                               eval(O, alpha[code_length-pos]) / eval(DLx, alpha[code_length-pos]);
                code.rep[pos] += _zz_pE__rep(e).rep[0];

                //std::cerr << "error/erasure at position " << pos << " val: " << -e << std::endl;
            }
        }
    }

    code_vec.resize(code_length);
    for (long i=0; i!=code_length; ++i)
    {
        code_vec[i] = NTL::rep(code.rep[i]);
    }



    for (long i=0; i!=end_range-start_range; ++i)
    {
        Syndrome(Sx.rep[i], code, alpha[start_range+i]);
    }
    Sx.normalize();
    if (!NTL::IsZero(Sx))
    {
        /*
        std::cerr << "Uncorrectable code\n";
        std::cerr << code << std::endl;
        std::cerr << Sx << std::endl;
        exit(2);
        */
        return 1;
    }
    return 0;
}



long Coding::generate(std::vector<long>& code, long index)
{
    index %= num_words;
    code.resize(code_length);
    for (long i=0; i!=code_length; ++i)
        code[i]=0L;
    long start=0L;
    while(index)
    {
        const long val=index%7;
        for (long i=0; i!=code_length; ++i)
            code[(start+i)%code_length] += (val*generator[i])%7;
        index /= 7;
        ++start;
    }
    for (long i=0; i!=code_length; ++i)
        code[i] %= 7;

    align(code,index,start);
    return index;
}

#if 0
int main()
{
    timeval tv;
    gettimeofday(&tv,0);
    srand((unsigned)tv.tv_usec);

    init();

    std::vector<long> code;
    ///*
    while(std::cin)
    {
        unsigned char c;
        if (!(std::cin >> c)) break;

        switch (c)
        {
        case '-' :
            code.push_back(-1L);
            break;
        case '0' :
        case '1' :
        case '2' :
        case '3' :
        case '4' :
        case '5' :
        case '6' :
            code.push_back(static_cast<long>(c-'0'));
        }
    }
    //*/
    /*
    long rand_idx=rand()%num_words;
    long idx=generate(code,rand_idx);
    */

    long index,rotation;
    decode(code);

    std::cout << "\ncorrected code:\n[";
    for (long i=0; i!=code_length; ++i)
        std::cout << " " << code[i];
    std::cout << " ]\n\n";


    align(code, index, rotation);

    std::cout << "Aligned code\nIndex: " << index << //", idx: " << idx << ", rand_idx: " << rand_idx <<
                 ", rotation: " << rotation << std::endl;

    std::cout << "[";
    for (long i=0; i!=code_length; ++i)
        std::cout << " " << code[i];
    std::cout << " ]\n\n";

    return 0;
}




#endif