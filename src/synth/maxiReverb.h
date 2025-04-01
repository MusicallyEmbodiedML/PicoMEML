/*
 Coded by Tom Rushmore,
 http:///github.com/tomrushmore
 */
/*
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

#ifndef __maxiReverb__
#define __maxiReverb__

#include "maximilian.h"
#include <vector>

template <size_t N>
class maxiReverbFilters{
public:
    maxiReverbFilters();
    float twopoint(float input);
    // float comb1(float input,float size);
    // float combff(float input,float size);
    // float combfb(float input,float size,float fb);
    // // float lpcombfb(float input,float size,float fb, float cutoff);

    // float allpass(float input,float size);
    float allpass(float input,float size,float fback);
    // float allpasstap(float input,float size,int tap);
    // void setlength(int length);
    // float onetap(float input,float size);
    // float tapd(float input,float size, float * taps,int numtaps);
    // float tapdwgain(float input,float size, float * taps,int numtaps,float * gain);
    // float tapdpos(float input,int size, int * taps,int numtaps);
    // float gettap(int tap);

private:
    std::vector<float> delay_line;
    float a;
    size_t delay_index=0;
    size_t delay_size;
    float output;
    float feedback;
    float gain_cof;

    // maxiFilter mf;


};

template <size_t N>
maxiReverbFilters<N>::maxiReverbFilters()
{
    a = 0.0;
    output = 0.0;
    delay_index = 0;
    feedback = 0.8;
    gain_cof = 0.85;
    delay_line.resize(N,0);
}

template <size_t N>
float maxiReverbFilters<N>::twopoint(float input)
{
    a = 0.5 * (input + a);
    // return a;
    return input;
}

// template <size_t N>
// float maxiReverbFilters<N>::comb1(float input,float size)
// {
//     delay_size = size;
//     output = delay_line[delay_index];
//     delay_line[delay_index] = input + (feedback * output);
//     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
//     return output;
// }


// template <size_t N>
// float maxiReverbFilters<N>::combff(float input, float size)
// {
//     delay_size = size;
//     output = input + delay_line[delay_index];
//     delay_line[delay_index] = input;
//     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
//     return output;
// }

// template <size_t N>
// float maxiReverbFilters<N>::combfb(float input, float size, float fb)
// {
//     // holding delay size allows me to tap line
//     delay_size = size;
//     output = input + (fb * delay_line[delay_index]);
//     delay_line[delay_index] = output;
//     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
//     return output;
// }

// // template <size_t N>
// // float maxiReverbFilters<N>::lpcombfb(float input, float size, float fb, float cutoff)
// // {
// //     // used for freeverb emulation
// //     // low pass between delay output + feedback
// //     delay_size = size;
// //     output = input + (fb * mf.lopass(delay_line[delay_index],(1.0-cutoff)));
// //     delay_line[delay_index] = output;
// //     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
// //     return output;
// //     return 0.0;
    
// // }

// template <size_t N>
// float maxiReverbFilters<N>::allpass(float input,float size)
// {
//     delay_size = size;
//     input += delay_line[delay_index] * gain_cof;
//     output = delay_line[delay_index] + (input * (-gain_cof));
//     delay_line[delay_index] = input;
//     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
    
//     return output;
// }

template <size_t N>
float maxiReverbFilters<N>::allpass(float input,float size,float fb)
{
    delay_size = size;
    input += delay_line[delay_index] * fb;
    output = delay_line[delay_index] + (input * (-fb));
    delay_line[delay_index] = input;
    delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
    return output;
}

// template <size_t N>
// float maxiReverbFilters<N>::allpasstap(float input,float size,int tap)
// {
//     delay_size = size;
//     input += delay_line[delay_index] * gain_cof;
    
//     int t = delay_index + tap;
//     if(t > delay_size -1){
//         t -= delay_size;
//     }
//     output = delay_line[t];
//     delay_line[delay_index] = input;
//     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
    
//     return output;
// }

// template <size_t N>
// float maxiReverbFilters<N>::gettap(int tap)
// {
//     int t = delay_index + tap;
//     if(t > delay_size -1){
//         t -= delay_size;
//     }
//     output = delay_line[t];
//     return output;
// }

// template <size_t N>
// float maxiReverbFilters<N>::onetap(float input, float size)
// {
//     delay_size = size;
//     output = delay_line[delay_index];
//     delay_line[delay_index] = input;
//     delay_index != size - 1 ? delay_index++ : delay_index = 0;
//     return output;
// }

// template <size_t N>
// float maxiReverbFilters<N>::tapd(float input,float size, float * taps,int numtaps)
// {
//     output = 0.0;
//     delay_size = size;
//     for(int i = 0; i < numtaps ; i++)
//     {
//         float t = (int)(taps[i] * (size-1));
//         int o = delay_index + t;
//         if(o > delay_size -1)
//             o -= delay_size;
//         output += delay_line[o];
//     }
//     delay_line[delay_index] = input;
//     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
//     return output;
// }

// template <size_t N>
// float maxiReverbFilters<N>::tapdwgain(float input,float size, float * taps,int numtaps,float * gain)
// {
//     output = 0.0;
//     delay_size = size;
//     for(int i = 0; i < numtaps ; i++)
//     {
//         float t = (int)(taps[i] * (delay_size-1));
//         int o = delay_index + t;
//         if(o > delay_size -1)
//             o -= delay_size;
//         output += gain[i] * delay_line[o];
//     }
//     delay_line[delay_index] = input;
//     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
//     return output;
// }

// template <size_t N>
// float maxiReverbFilters<N>::tapdpos(float input,int size, int * taps,int numtaps)
// {
//     output = 0.0;
//     delay_size = size;
//     for(int i = 0; i < numtaps ; i++)
//     {
//         output += delay_line[taps[i]];
//     }
//     delay_line[delay_index] = input;
//     delay_index != delay_size - 1 ? delay_index++ : delay_index = 0;
//     return output;
// }




#endif /* defined(__maximilianZone__maxiReverb__) */

