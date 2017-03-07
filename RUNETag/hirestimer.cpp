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

#include "hirestimer.hpp"


HiresTimer::HiresTimer(void)
{
	started = false;
	stopped = true;
}

HiresTimer::~HiresTimer(void)
{
}


#ifdef WIN32
/*
 *	Win32 Hi-res timer code
 */

double LI1D(LARGE_INTEGER *i) { 
return(i->LowPart+(i->HighPart*4294967296.0)); 
} 

void HiresTimer::start() {
	started = true;
	stopped = false;
	
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start_time);
}

double HiresTimer::elapsed() {
	if( !started )
		return 0.0;

	if( !stopped ) {
		LARGE_INTEGER end_time;
		QueryPerformanceCounter(&end_time);
		elapsed_time = (LI1D(&end_time) - LI1D(&start_time)) / LI1D(&frequency);
	}
	return elapsed_time;
}

void HiresTimer::stop() {
	LARGE_INTEGER end_time;
	QueryPerformanceCounter(&end_time);
	elapsed_time = (LI1D(&end_time) - LI1D(&start_time)) / LI1D(&frequency);
	stopped = true;
}

void HiresTimer::reset() {
	started = false;
	stopped = false;
}


#else

#ifdef UNIX
/*
 *	Unix Hi-res timer code
 */

void HiresTimer::start() {
	started = true;
	stopped = false;
	start_time = 0.0;
	struct timeval start_timeval;
	gettimeofday( &start_timeval, NULL );
	start_time = (double)start_timeval.tv_sec + (double)start_timeval.tv_usec/1000000.0;
}

double HiresTimer::elapsed() {
	if( !started )
		return 0.0;

	if( !stopped ) {
		struct timeval end_timeval;
		gettimeofday( &end_timeval, NULL );
		double tnow = (double)end_timeval.tv_sec + (double)end_timeval.tv_usec/1000000.0;
		elapsed_time = tnow-start_time;
	}
	return elapsed_time;
}

void HiresTimer::stop() {
	struct timeval end_timeval;
	gettimeofday( &end_timeval, NULL );
	double tnow = (double)end_timeval.tv_sec + (double)end_timeval.tv_usec/1000000.0;
	elapsed_time = tnow-start_time;
	stopped = true;
}

void HiresTimer::reset() {
	started = false;
	stopped = false;
}


#else
 /*
  *  Standard timer (NOT yet implemented)
  */
//#error Default standard timer not yet implemented. You see this error probably because you are trying to compile against an unsupported platform
#endif

#endif
