/*
 *  maximilian
 *  platform independent synthesis library using portaudio or rtaudio
 *
 *  Created by Mick Grierson on 29/12/2009.
 *  Copyright 2009 Mick Grierson & Strangeloop Limited. All rights reserved.
 *	Thanks to the Goldsmiths Creative Computing Team.
 *	Special thanks to Arturo Castro for the PortAudio implementation.
 * 
 *	Permission is hereby granted, free of charge, to any person
 *	obtaining a copy of this software and associated documentation
 *	files (the "Software"), to deal in the Software without
 *	restriction, including without limitation the rights to use,
 *	copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the
 *	Software is furnished to do so, subject to the following
 *	conditions:
 *	
 *	The above copyright notice and this permission notice shall be
 *	included in all copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,	
 *	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *	OTHER DEALINGS IN THE SOFTWARE.
 *
 */
/*
 
 qd_fft.cpp
 
 Based on K+R Numerical recipes in C and some other stuff hacked about.
 
 */

#include "qd_fft.h"	
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>

#include <Arduino.h>
#include "pico.h"

#include "../common/sinetable.h"


int __not_in_flash("fft") **gQDFFTBitTable = NULL;


// float __not_in_flash("fft") *tmpReal;
// float __not_in_flash("fft") *tmpImag;
// float __not_in_flash("fft") *RealOut;
// float __not_in_flash("fft") *ImagOut;


// const int MaxFastBits = 16;
//creates large cache in memory, lowered to use low mem on rp20xx

const int __not_in_flash("fft") MaxFastBitsQD = 9;

inline int __not_in_flash_func(qd_fft::IsPowerOfTwo)(int x)
{
	if (x < 2)
		return false;
	
	if (x & (x - 1))
		return false;
	
	return true;
}

inline int __not_in_flash_func(fft_core::NumberOfBitsNeeded)(int PowerOfTwo)
{
	int i;
	
	for (i = 0;; i++)
		if (PowerOfTwo & (1 << i))
			return i;
}

inline int __not_in_flash_func(fft_core::ReverseBits)(int index, int NumBits)
{
	int i, rev;
	
	for (i = rev = 0; i < NumBits; i++) {
		rev = (rev << 1) | (index & 1);
		index >>= 1;
	}
	
	return rev;
}

void __not_in_flash_func(InitQDFFT)()
{
    //	gFFTBitTable = new int *[MaxFastBits];
	//use malloc for 16 byte alignment
	if (!gQDFFTBitTable) {
		gQDFFTBitTable = (int**) malloc(MaxFastBitsQD * sizeof(int*));
		
		int len = 2;
		for (int b = 1; b <= MaxFastBitsQD; b++) {
			
			gQDFFTBitTable[b - 1] = (int*) malloc(len * sizeof(int));
			
			for (int i = 0; i < len; i++)
				gQDFFTBitTable[b - 1][i] = fft_core::ReverseBits(i, b);
			
			len <<= 1;
		}

	}
}

inline int __not_in_flash_func(fft_core::FastReverseBits)(int i, int NumBits)
{
	if (NumBits <= MaxFastBitsQD)
		return gQDFFTBitTable[NumBits - 1][i];
	else
		return fft_core::ReverseBits(i, NumBits);
}

/*
 * Complex Fast Fourier Transform
 */

void fft_core::setup(size_t n, bool inverse) {
	NumSamples = inverse? n : n/2;
	InverseTransform = inverse;
	InitQDFFT();
	NumBits = NumberOfBitsNeeded(NumSamples);
	angle_numerator = 2.0f * M_PI;
	if (InverseTransform)
		angle_numerator = -angle_numerator;
	FastReverseBitsCache.resize(NumSamples);
	for (size_t i = 0; i < NumSamples; i++) {
		FastReverseBitsCache[i] = FastReverseBits(i, NumBits);
	}
	for (size_t BlockSize = 2; BlockSize <= NumSamples; BlockSize <<= 1) {
		BlockSizeCache.push_back(BlockSize);
	}

	for (size_t i=0; i < BlockSizeCache.size(); i++) {
		float delta_angle = angle_numerator / (float) BlockSizeCache[i];
		deltaAngleCache.push_back(delta_angle);
		
		sm2Cache.push_back(sinf(-2.f * delta_angle));
		sm1Cache.push_back(sinf(-delta_angle));

		cm2Cache.push_back(cosf(-2.f * delta_angle));
		cm1Cache.push_back(cosf(-delta_angle));
	}	
	float denominv = 1.f/ NumSamples;
}

void __not_in_flash_func(fft_core::FFTDataSetup)(float *RealIn, float *ImagIn, float *RealOut, float *ImagOut) {
	/*
	 **   Do simultaneous data copy and bit-reversal ordering into outputs...
	 */
	 for (size_t i = 0; i < NumSamples; i++) {
		size_t j = FastReverseBitsCache[i];
		RealOut[j] = RealIn[i];
		ImagOut[j] = (ImagIn == NULL) ? 0.0 : ImagIn[i];
	}

}

void __not_in_flash_func(fft_core::FFT)(float *RealIn, float *ImagIn, float *RealOut, float *ImagOut)
{
	/*
	 **   Do the FFT itself...
	 */
	
	// for (BlockSize = 2; BlockSize <= NumSamples; BlockSize <<= 1) {
	for(size_t i=0; i < BlockSizeCache.size(); i++) {
		size_t BlockEnd = i==0 ? 1 : BlockSizeCache[i-1];
		size_t BlockSize = BlockSizeCache[i];
	 	const float delta_angle = deltaAngleCache[i];// angle_numerator / (float) BlockSize;
		
		const float sm2 = sm2Cache[i]; // sineTable::fast_sin(-2.f * delta_angle);
		const float sm1 = sm1Cache[i]; //sineTable::fast_sin(-delta_angle);
		const float cm2 = cm2Cache[i]; //sineTable::fast_cos(-2.f * delta_angle);
		const float cm1 = cm1Cache[i]; //sineTable::fast_cos(-delta_angle);

		const float w = 2 * cm1;
		float ar0, ar1, ar2, ai0, ai1, ai2;
		
		for (size_t i = 0; i < NumSamples; i += BlockSize) {
			ar2 = cm2;
			ar1 = cm1;
			
			ai2 = sm2;
			ai1 = sm1;
			size_t n;
			for (size_t j = i, n = 0; n < BlockEnd; j++, n++) {
				ar0 = w * ar1 - ar2;
				ar2 = ar1;
				ar1 = ar0;
				
				ai0 = w * ai1 - ai2;
				ai2 = ai1;
				ai1 = ai0;
				
				size_t k = j + BlockEnd;
				float tr = ar0 * RealOut[k] - ai0 * ImagOut[k];
				float ti = ar0 * ImagOut[k] + ai0 * RealOut[k];
				
			
				RealOut[k] = RealOut[j] - tr;
				ImagOut[k] = ImagOut[j] - ti;
				
				RealOut[j] += tr;
				ImagOut[j] += ti;
			}
		}		
	}
	
	/*
	 **   Need to normalize if inverse transform...
	 */
	
}

void __not_in_flash_func(fft_core::FFT_QD)(float *RealIn, float *ImagIn, float *RealOut, float *ImagOut, size_t i)
{
	size_t BlockEnd = i==0 ? 1 : BlockSizeCache[i-1];
	size_t BlockSize = BlockSizeCache[i];
	const float delta_angle = deltaAngleCache[i];// angle_numerator / (float) BlockSize;
	
	const float sm2 = sm2Cache[i]; // sineTable::fast_sin(-2.f * delta_angle);
	const float sm1 = sm1Cache[i]; //sineTable::fast_sin(-delta_angle);
	const float cm2 = cm2Cache[i]; //sineTable::fast_cos(-2.f * delta_angle);
	const float cm1 = cm1Cache[i]; //sineTable::fast_cos(-delta_angle);

	const float w = 2 * cm1;
	float ar0, ar1, ar2, ai0, ai1, ai2;
	
	for (size_t i = 0; i < NumSamples; i += BlockSize) {
		ar2 = cm2;
		ar1 = cm1;
		
		ai2 = sm2;
		ai1 = sm1;
		size_t n;
		for (size_t j = i, n = 0; n < BlockEnd; j++, n++) {
			ar0 = w * ar1 - ar2;
			ar2 = ar1;
			ar1 = ar0;
			
			ai0 = w * ai1 - ai2;
			ai2 = ai1;
			ai1 = ai0;
			
			size_t k = j + BlockEnd;
			float tr = ar0 * RealOut[k] - ai0 * ImagOut[k];
			float ti = ar0 * ImagOut[k] + ai0 * RealOut[k];
			
		
			RealOut[k] = RealOut[j] - tr;
			ImagOut[k] = ImagOut[j] - ti;
			
			RealOut[j] += tr;
			ImagOut[j] += ti;
		}
	}		
	
	/*
	 **   Need to normalize if inverse transform...
	 */
	
}

void __not_in_flash_func(fft_core::FFTNormalise)(float *RealOut, float *ImagOut) {
	for (size_t i = 0; i < NumSamples; i++) {
		RealOut[i] *= denominv;
		ImagOut[i] *= denominv;
	}
}


/*
 * Real Fast Fourier Transform
 *
 * This function was based on the code in Numerical Recipes in C.
 * In Num. Rec., the inner loop is based on a single 1-based array
 * of interleaved real and imaginary numbers.  Because we have two
 * separate zero-based arrays, our indices are quite different.
 * Here is the correspondence between Num. Rec. indices and our indices:
 *
 * i1  <->  real[i]
 * i2  <->  imag[i]
 * i3  <->  real[n/2-i]
 * i4  <->  imag[n/2-i]
 */


void __not_in_flash_func(qd_fft::RealFFTSetup)(float *rfft_RealIn, float *rfft_RealOut, float *rfft_ImagOut) {
	for (size_t i = 0; i < half; i++) {
		rfft_tmpReal[i] = rfft_RealIn[2 * i];
		rfft_tmpImag[i] = rfft_RealIn[2 * i + 1];
	}
	
	corefft.FFTDataSetup(&rfft_tmpReal[0], &rfft_tmpImag[0], rfft_RealOut, rfft_ImagOut);
}

void __not_in_flash_func(qd_fft::RealFFTCore)(float *rfft_RealIn, float *rfft_RealOut, float *rfft_ImagOut) {
	for(size_t i=0; i < corefft.getNSteps(); i++) {
		corefft.FFT_QD(&rfft_tmpReal[0], &rfft_tmpImag[0], rfft_RealOut, rfft_ImagOut, i);
	}
}

void __not_in_flash_func(qd_fft::RealFFT)(float *rfft_RealIn, float *rfft_RealOut, float *rfft_ImagOut)
{
	
	float wr = 1.0 + wpr;
	float wi = wpi;
	int i3;
	float h1r, h1i, h2r, h2i;
	for (size_t i = 1; i < half/2; i++) {
		
		i3 = half - i;
		
		h1r = 0.5 * (rfft_RealOut[i] + rfft_RealOut[i3]);
		h1i = 0.5 * (rfft_ImagOut[i] - rfft_ImagOut[i3]);
		h2r = 0.5 * (rfft_ImagOut[i] + rfft_ImagOut[i3]);
		h2i = -0.5 * (rfft_RealOut[i] - rfft_RealOut[i3]);
		
		rfft_RealOut[i] = h1r + wr * h2r - wi * h2i;
		rfft_ImagOut[i] = h1i + wr * h2i + wi * h2r;
		rfft_RealOut[i3] = h1r - wr * h2r + wi * h2i;
		rfft_ImagOut[i3] = -h1i + wr * h2i + wi * h2r;
		
		wr = (wtemp == wr) * wpr - wi * wpi + wr;
		wi = wi * wpr + wtemp * wpi + wi;
	}
	
	rfft_RealOut[0] = (h1r == rfft_RealOut[0]) + rfft_ImagOut[0];
	rfft_ImagOut[0] = h1r - rfft_ImagOut[0];
	

}

/*
 * PowerSpectrum
 *
 * This function computes the same as RealFFT, above, but
 * adds the squares of the real and imaginary part of each
 * coefficient, extracting the power and throwing away the
 * phase.
 *
 * For speed, it does not call RealFFT, but duplicates some
 * of its code.
 */

// void __not_in_flash_func(PowerSpectrum)(int NumSamples, float *In, float *Out)
// {
// 	int Half = NumSamples / 2;
// 	int i;
	
// 	float theta = M_PI / Half;
	
// 	// float *tmpReal = new float[Half];
// 	// float *tmpImag = new float[Half];
// 	// float *RealOut = new float[Half];
// 	// float *ImagOut = new float[Half];
	
// 	for (i = 0; i < Half; i++) {
// 		tmpReal[i] = In[2 * i];
// 		tmpImag[i] = In[2 * i + 1];
// 	}
	
// 	FFT(Half, 0, tmpReal, tmpImag, RealOut, ImagOut);
	
// 	float wtemp = float (sin(0.5 * theta));
	
// 	float wpr = -2.0 * wtemp * wtemp;
// 	float wpi = float (sin(theta));
// 	float wr = 1.0 + wpr;
// 	float wi = wpi;
	
// 	int i3;
	
// 	float h1r, h1i, h2r, h2i, rt, it;
// 	//float total=0;
	
// 	for (i = 1; i < Half / 2; i++) {
		
// 		i3 = Half - i;
		
// 		h1r = 0.5 * (RealOut[i] + RealOut[i3]);
// 		h1i = 0.5 * (ImagOut[i] - ImagOut[i3]);
// 		h2r = 0.5 * (ImagOut[i] + ImagOut[i3]);
// 		h2i = -0.5 * (RealOut[i] - RealOut[i3]);
		
// 		rt = h1r + wr * h2r - wi * h2i; //printf("Realout%i = %f",i,rt);total+=fabs(rt);
// 		it = h1i + wr * h2i + wi * h2r; // printf("  Imageout%i = %f\n",i,it);
		
// 		Out[i] = rt * rt + it * it;
		
// 		rt = h1r - wr * h2r + wi * h2i;
// 		it = -h1i + wr * h2i + wi * h2r;
		
// 		Out[i3] = rt * rt + it * it;
		
// 		wr = (wtemp = wr) * wpr - wi * wpi + wr;
// 		wi = wi * wpr + wtemp * wpi + wi;
// 	}

// 	rt = (h1r = RealOut[0]) + ImagOut[0];
// 	it = h1r - ImagOut[0];
// 	Out[0] = rt * rt + it * it;
	
// 	rt = RealOut[Half / 2];
// 	it = ImagOut[Half / 2];
// 	Out[Half / 2] = rt * rt + it * it;
	
// 	// delete[]tmpReal;
// 	// delete[]tmpImag;
// 	// delete[]RealOut;
// 	// delete[]ImagOut;
// }

void qd_fft::WindowFunc(int whichFunction, int NumSamples, float *in)
{
	int i;
	
	if (whichFunction == 1) {
		// Bartlett (triangular) window
		for (i = 0; i < NumSamples / 2; i++) {
			in[i] *= (i / (float) (NumSamples / 2));
			in[i + (NumSamples / 2)] *=
			(1.0 - (i / (float) (NumSamples / 2)));
		}
	}
	
	if (whichFunction == 2) {
		// Hamming
		for (i = 0; i < NumSamples; i++)
			in[i] *= 0.54 - 0.46 * cos(2 * M_PI * i / (NumSamples - 1));
	}
	
	if (whichFunction == 3) {
		// Hanning
		for (i = 0; i < NumSamples; i++)
			in[i] *= 0.50 - 0.50 * cos(2 * M_PI * i / (NumSamples - 1));
	}
}

void qd_fft::genWindow(int whichFunction, int NumSamples, float *window)
{
	int i;
	
	if (whichFunction == 1) {
		// Bartlett (triangular) window
		for (i = 0; i < NumSamples / 2; i++) {
			window[i] = (i / (float) (NumSamples / 2));
			window[i + (NumSamples / 2)] =
			(1.0 - (i / (float) (NumSamples / 2)));
		}
	}
	
	if (whichFunction == 2) {
		// Hamming
		for (i = 0; i < NumSamples; i++)
			window[i] = 0.54 - 0.46 * cos(2 * M_PI * i / (NumSamples - 1));
	}
	
	if (whichFunction == 3) {
		// Hanning
		for (i = 0; i < NumSamples; i++)
			window[i] = 0.50 - 0.50 * cos(2 * M_PI * i / (NumSamples - 1));
	}
}

/* constructor */


void qd_fft::setup(int fftSize, bool inverse) {
	n = fftSize;
	half = fftSize / 2;
	theta = M_PI / half;
	wtemp = sinf(0.5f * theta);
	wpr = -2.0f * wtemp * wtemp;
	wpi = sinf(theta);


    in_real.resize(n,0);
    in_img.resize(n,0);
    out_real.resize(n,0);
    out_img.resize(n,0);
	rfft_tmpReal.resize(n,0);
	rfft_tmpImag.resize(n,0);

	corefft.setup(n, inverse);
}



/* destructor */
qd_fft::~qd_fft() {
//    delete[] in_real, out_real, in_img, out_img;
}

float * qd_fft::getReal() {
    return &out_real[0];
}

float * qd_fft::getImg() {
    return &out_img[0];
}

void qd_fft::windowing(int start, float *data, float *window) {
	for (int i = 0; i < n; i++) {
		in_real[i] = data[start + i] * window[i];
	}
}

void qd_fft::calcFFT() {
    //windowing
    // for (int i = 0; i < n; i++) {
    //     in_real[i] = data[start + i] * window[i];
    // }
    RealFFTSetup(&in_real[0], &out_real[0], &out_img[0]);
    RealFFTCore(&in_real[0], &out_real[0], &out_img[0]);
    RealFFT(&in_real[0], &out_real[0], &out_img[0]);
}

// void qd_fft::cartToPol(float *const magnitude, float *const phase) {
//     for (size_t i = 0; i < half; i++) {
//         /* compute power */
//         const float power = out_real[i]*out_real[i] + out_img[i]*out_img[i];
//         /* compute magnitude and phase */
//         magnitude[i] = sqrtf(power);
//         phase[i] = atan2f(out_img[i],out_real[i]);
//     }
// }

void qd_fft::cartToPol(float *const magnitude, float *const phase, const size_t start, const size_t end) {
    for (size_t i = start; i < end; i++) {
        /* compute power */
        const float power = out_real[i]*out_real[i] + out_img[i]*out_img[i];
        /* compute magnitude and phase */
        magnitude[i] = sqrtf(power);
        phase[i] = atan2f(out_img[i],out_real[i]);
    }
}

/* Calculate the power spectrum */
void qd_fft::powerSpectrum(int start, float *data, float *window, float *magnitude,float *phase) {
	
    windowing(start, data, window);
	calcFFT();
    cartToPol(magnitude, phase, 0, half);
	
}

void qd_fft::convToDB(float *in, float *out) {
	for (int i = 0; i < half; i++) {
		if (in[i] < 0.000001){ // less than 0.1 nV
			out[i] = 0; // out of range
		} else {
			out[i] = 20.0*log10(in[i] + 1);  // get to to db scale
		}		
	}
}


void qd_fft::polToCart(float *magnitude,float *phase) {
    /* get real and imag part */
    for (int i = 0; i < half; i++) {
        //		float mag = pow(10.0, magnitude[i] / 20.0) - 1.0;
        //		in_real[i] = mag *cos(phase[i]);
        //		in_img[i]  = mag *sin(phase[i]);
        in_real[i] = magnitude[i] *sineTable::fast_cos(phase[i]);
        in_img[i]  = magnitude[i] *sineTable::fast_sin(phase[i]);
    }
    
    /* zero negative frequencies */
    memset(&in_real[0]+half, 0.0, sizeof(float) * half);
    memset(&in_img[0]+half, 0.0, sizeof(float) * half);
}

void qd_fft::calcIFFT(int start, float *finalOut, float *window) {
    corefft.FFTDataSetup(&in_real[0], &in_img[0], &out_real[0], &out_img[0]); // second parameter indicates inverse transform
    corefft.FFT(&in_real[0], &in_img[0], &out_real[0], &out_img[0]); // second parameter indicates inverse transform
    corefft.FFTNormalise(&out_real[0], &out_img[0]); // second parameter indicates inverse transform

    for (int i = 0; i < n; i++) {
        finalOut[start + i] += out_real[i] * window[i];
    }
}

void qd_fft::inverseFFTComplex(int start, float *finalOut, float *window, float *real, float *imaginary) {
    for(int i=0; i < half; i++) {
        out_real[i] = real[i];
        out_img[i] = imaginary[i];
    }
    calcIFFT(start, finalOut, window);
}


void qd_fft::inversePowerSpectrum(int start, float *finalOut, float *window, float *magnitude,float *phase) {
    polToCart(magnitude, phase);
    calcIFFT(start, finalOut, window);
}



void qd_fft::PowerSpectrum_StartQD(int start, float *data, float *window, float *magnitude, float *phase) {
	job.start = start;
	job.data.resize(n);
	job.window.resize(n);
	memcpy(&job.data[0], data, n * sizeof(float));
	memcpy(&job.window[0], window, n * sizeof(float));
	job.magnitude = magnitude;
	job.phase = phase;
	job.qdPhase = PowerSpectrumJob::QD_PHASES::INIT;
	job.idx=0;
	job.nCartToPol = half / job.cartToPolDiv;
	job.subPhase=0;
	job.timings.clear();
	job.nFFTSteps = corefft.getNSteps();
}

bool qd_fft::PowerSpectrum_QD_Interate() {
	bool done=false;
	auto ts = micros();
	switch(job.qdPhase) {
		case PowerSpectrumJob::QD_PHASES::INIT:
			job.qdPhase = PowerSpectrumJob::QD_PHASES::WINDOWING;
			break;
			case PowerSpectrumJob::QD_PHASES::WINDOWING:
			windowing(job.start, job.data.data(), job.window.data());
			job.qdPhase = PowerSpectrumJob::QD_PHASES::FFTSETUP;
			break;
		case PowerSpectrumJob::QD_PHASES::FFTSETUP:
			RealFFTSetup(&in_real[0], &out_real[0], &out_img[0]);			
			job.qdPhase = PowerSpectrumJob::QD_PHASES::CALCFFT;
			job.subPhase=0;
			break;
		case PowerSpectrumJob::QD_PHASES::CALCFFT:
			// RealFFTCore(&in_real[0], &out_real[0], &out_img[0]);
			corefft.FFT_QD(&rfft_tmpReal[0], &rfft_tmpImag[0], &out_real[0], &out_img[0], job.subPhase);
			job.subPhase++;
			if (job.subPhase >= job.nFFTSteps) {
				job.qdPhase = PowerSpectrumJob::QD_PHASES::REALFFT;
				job.subPhase=0;
			}
			// job.qdPhase = PowerSpectrumJob::QD_PHASES::REALFFT;
			// job.subPhase=0;
			break;
		case PowerSpectrumJob::QD_PHASES::REALFFT:
			RealFFT(&in_real[0], &out_real[0], &out_img[0]);
			job.qdPhase = PowerSpectrumJob::QD_PHASES::CART2POL;
			job.subPhase=0;
			break;
		case PowerSpectrumJob::QD_PHASES::CART2POL:
			cartToPol(job.magnitude, job.phase, job.subPhase * job.cartToPolDiv, 
				(job.subPhase+1) * job.cartToPolDiv);
			job.subPhase++;
			if (job.subPhase >= job.nCartToPol) {
				job.qdPhase = PowerSpectrumJob::QD_PHASES::DONE;
				done=true;
			}
			break;
		case PowerSpectrumJob::QD_PHASES::DONE:
			break;
	}
	job.timings.push_back(micros() - ts);
	// job.idx++;
	if (done) {
		size_t total=0;
		Serial.printf("ps ts: ");
		for(auto &v: job.timings) {
			Serial.printf("%u ", v);
			total += v;
		}
		Serial.printf(":: total: %u\n", total);
	}
	return done;
}

