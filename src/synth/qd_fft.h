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

#ifndef _FFT
#define _FFT

#ifndef M_PI
#define	M_PI		3.14159265358979323846  /* pi */
#endif
#include <vector>
#include <cstdint>

class fft_core {
public:

    struct FFTStates {
        float ar0, ar1, ar2, ai0, ai1, ai2;
        size_t block=0, idx=0, j=0, n=0;
        size_t iterationCount=0;
        size_t maxIterationsPerQuanta=64;
    } st;

    void initFFT_QD_State(size_t maxItPerQ) {
        st.block =0;
        st.idx=0;
        st.j=0;
        st.n=0;
        st.maxIterationsPerQuanta = maxItPerQ;
    }

    void setup(size_t nSamples, bool inverse);
    void FFTDataSetup(float *RealIn, float *ImagIn, float *RealOut, float *ImagOut);
    void FFT(float *RealIn, float *ImagIn, float *RealOut, float *ImagOut);
    void FFT_QD(float *RealIn, float *ImagIn, float *RealOut, float *ImagOut, const size_t i, const size_t start, const size_t end);
    bool FFT_QD_B(float *RealIn, float *ImagIn, float *RealOut, float *ImagOut);
    void FFTNormalise(float *RealOut, float *ImagOut);

    int NumberOfBitsNeeded(int PowerOfTwo);
    int FastReverseBits(int i, int NumBits);
    static int ReverseBits(int index, int NumBits);

    size_t NumSamples;
    bool InverseTransform;

    size_t getNSteps() {
        return BlockSizeCache.size();
    }
private:
    int NumBits;                 /* Number of bits needed to store indices */
	float angle_numerator;
    std::vector<size_t> FastReverseBitsCache, BlockSizeCache;
    std::vector<float> deltaAngleCache, cm1Cache, cm2Cache, sm1Cache, sm2Cache;
    float denominv;

};

class qd_fft {
	
public:

    

    qd_fft(){};
	~qd_fft();
	
    std::vector<size_t> phaseTimings;
    fft_core corefft;

    void setup(int fftSize, bool inverse);
	int n; //fftSize
	int half; //halfFFTSize
    float theta, wtemp, wpr, wpi;

    std::vector<float> in_real,out_real,in_img,out_img;

    std::vector<float>  rfft_tmpReal, rfft_tmpImag;
    
    
    float * getReal();
    float * getImg();

    
	/* Calculate the power spectrum */
    void windowing(int start, float *data, float *window);

    void calcFFT();
    void cartToPol(float *magnitude, float *phase, const size_t start, const size_t end);
	void powerSpectrum(int start, float *data, float *window, float *magnitude, float *phase);
	/* ... the inverse */
    void polToCart(float *const magnitude,float *const phase, const size_t start, const size_t end);
    void zeroNegFreqs();
    void calcIFFT(int start, float *finalOut, float *window);
    void inverseFFTComplex(int start, float *finalOut, float *window, float *real, float *imaginary);
	void inversePowerSpectrum(int start, float *finalOut, float *window, float *magnitude,float *phase);
	void convToDB(float *in, float *out);
    void iFFTWindowing(int start, float *finalOut, float *window);
    
	static void genWindow(int whichFunction, int NumSamples, float *window);
    void RealFFTSetup(float *rfft_RealIn, float *rfft_RealOut, float *rfft_ImagOut);
    void RealFFTCore(float *RealIn, float *RealOut, float *ImagOut);
    void RealFFT(float *RealIn, float *RealOut, float *ImagOut);


    struct PowerSpectrumJob {
        enum QD_PHASES : uint8_t {INIT=0, WINDOWING, FFTSETUP, CALCFFT, REALFFT, CART2POL, DONE} qdPhase;
        int start;
        std::vector<float> data;
        std::vector<float> window;
        float *magnitude;
        float *phase;
        std::vector<size_t> timings;
        size_t idx;

        size_t cartToPolDiv=32;
        size_t nCartToPol;
        size_t subPhase=0;
        size_t nFFTSteps;

    };
	PowerSpectrumJob job;

    void PowerSpectrum_StartQD(int start, float *data, float *window, float *magnitude, float *phase);
    bool PowerSpectrum_QD_Interate();


    struct InvPowerSpectrumJob {
        enum QD_PHASES : uint8_t {INIT=0, POL2CART, NEGFREQS, INVFFTSETUP, CALCINVFFT, IFFTNORMALISE, IFFTWINDOWING, DONE} qdPhase;
        int start;
        float * sigout;
        float * window;
        std::vector<float> magnitude;
        std::vector<float> phase;
        std::vector<size_t> timings;

        size_t Pol2CartDiv=32;
        size_t nPolToCart;
        size_t subPhase=0;

        size_t nFFTSteps;
        size_t nFFTSubSteps;
        size_t nFFTSubStepDiv = 256;

    };
	InvPowerSpectrumJob invjob;

    void InvPowerSpectrum_StartQD(int start, float *sigout, float *window, float *magnitude, float *phase);
    bool InvPowerSpectrum_QD_Interate();

private:
    void WindowFunc(int whichFunction, int NumSamples, float *in);
    int IsPowerOfTwo(int x);



};


#endif	
