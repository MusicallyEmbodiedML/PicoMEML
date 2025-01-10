#pragma once

//#define NDEBUG
#include <cassert>
#include <vector>
#include <array>
#include "maximilian.h"
#include "OnePoleSmoother.hpp"

#define PERIODIC_DEBUG(COUNT, FUNC) \
        static size_t ct=0; \
        if (ct++ % COUNT == 0) { \
            FUNC         \
        }

const size_t kN_synthparams = 30;

inline float vox_fasttanh_ultra( const float x )
{
	const float ax = fabs( x );
	const float x2 = x * x;
	const float z = x * ( 0.773062670268356 + ax +
		( 0.757118539838817 + 0.0139332362248817 * x2 * x2 ) *
		x2 * ax );

	return( z / ( 0.795956503022967 + fabs( z )));
}

template<size_t N>
class matrixMixer {
public:
    matrixMixer() {
        //clear matrix
        for (auto &v: mixingMatrix) {
            v = 0.f;
        }
        scale = 1.f/N;
    }

    void randomise_linear(float low, float high) {
        float randMaxInv = 1.f / RAND_MAX;
        for(auto &v: mixingMatrix) {
            v = ((rand() * randMaxInv) * (high-low)) + low;
            v *= scale;
        }
    }

    void set(std::vector<float> &newValues) {
        // assert(newValues.size() < mixingMatrix.size());
        for(size_t i=0; i < mixingMatrix.size(); i++) {
            mixingMatrix[i] = ((newValues[i] * 2.f)-1.f) * scale;
        }
        // PERIODIC_DEBUG(6000,
        //     for(size_t i=0; i < mixingMatrix.size(); i++) {
        //         printf("%f\t", newValues[i]);
        //     }
        //     printf("\n");
        // )
    }

    float calculateMix(const std::array<float,N> &inputs, const size_t outputIndex) {
        assert(inputs.size() == N);
        float sum=0;
        size_t offset = N * outputIndex;
        for(size_t i=0; i < N; i++) {
            float mixValue = inputs[i] * mixingMatrix[offset+i];
            if (i == outputIndex) {
                mixValue *= directFeedbackScale;
            }
            sum += mixValue;

        }
        // PERIODIC_DEBUG(3001,
        //     printf("%u\t %u\t %f\t", outputIndex, mixingMatrix.size(),  sum);
        //     for(size_t i=0; i < N; i++) {
        //         printf("%f\t", mixingMatrix[offset+i]);
        //     };
        //     printf("\n");
        // )
        return sum;
    }

    void setDirectFeedbackScaling(const float scale) {
        directFeedbackScale = scale;
    }

    void scaleWithEigenValues(float alpha) {
        //is this needed?
    }

private:
    std::array<float, N*N> mixingMatrix;
    float directFeedbackScale = 0.1f;
    float scale=1;
    
};

#define NFX 4


class MaxtrixMixApp {
public:

    MaxtrixMixApp(size_t sample_rate): smoother_(100.f, sample_rate) {
        mmix.setDirectFeedbackScaling(0.0);
        unsmoothParams.resize(kN_synthparams);
        params.resize(kN_synthparams);
                
    }

    static void GenParams(std::vector<float> &param_vector)
    {
        float rand_scale = 1.f / static_cast<float>(RAND_MAX);

        for(size_t i=0; i < kN_synthparams; i++) {
            param_vector[i] = std::rand() * rand_scale;
        }
        //printf("\n");
    }

    float play(float x) {
        smoother_.Process(unsmoothParams.data(), params.data());

        // float flange = flanger.flange(x, params[3] * 6000 + 100, params[4] * 0.98, params[5] * 10.f, params[6]) * params[1];
        // float dist = distortion.fastAtanDist(x, params[7] * 10.0) * params[2] * 0.5;
        // float delayed = dl.play(x, params[8] * 10000 + 100, params[9] * 0.99) * params[0];
        // x = (flange + dist + delayed);
        // return x;
        mmix.set(params);
        const size_t ofs = NFX*NFX; //offset from mixer params
        fxInputs[0] = x * params[ofs+0] * params[ofs+0];
        fxInputs[1] = x * params[ofs+1] * params[ofs+1];
        fxInputs[2] = x * params[ofs+2] * params[ofs+2];
        fxInputs[3] = x * params[ofs+3] * params[ofs+3];


        float flangeInput = (mmix.calculateMix(fxOutputs, 0) + fxInputs[0]);
        float flange = flanger.flange(flangeInput, params[ofs+4] * 10000 + 100, params[ofs+5] * 0.99, params[ofs+6] * 0.99f, params[ofs+7]);

        // // float distInput = (mmix.calculateMix(fxOutputs, 1) + fxInputs[1]) * 0.5;
        // // float dist = distortion.fastAtanDist(distInput, params[ofs+8] * 2);
        

        float filterInput = (mmix.calculateMix(fxOutputs, 1) + fxInputs[1]) * 0.5;
        float filtered = filt.play(filterInput);

        // // PERIODIC_DEBUG(3000,
        // //     printf("%f\t%f\t%f\n", rmMod, rmInput, params[ofs+8]);
        // // )
        // float delayInput2 = (mmix.calculateMix(fxOutputs,1) + fxInputs[1]) * 0.5;
        // float delayed2 = dl.play(delayInput2, params[ofs+3] * 10000 + 100, params[ofs+8] * 0.95);


        float delayInput = (mmix.calculateMix(fxOutputs, 2) + fxInputs[2]) * 0.5;
        float delayed = dl.play(delayInput, params[ofs+9] * 10000 + 100, params[ofs+10] * 0.95);


        float rmMod = sinosc.sinebuf(1.f + (params[ofs+11] * params[ofs+11] * 800));
        float rmInput = (mmix.calculateMix(fxOutputs, 3) + fxInputs[3]) * 0.5;
        float rmSig = rmInput * rmMod;

        fxOutputs[0] = flange;
        fxOutputs[1] = filtered;
        fxOutputs[2] = delayed;
        fxOutputs[3] = rmSig;

        // x = fxOutputs[0] + fxOutputs[1] + fxOutputs[2];
        x = fxOutputs[0] + fxOutputs[1] + fxOutputs[2] + fxOutputs[3];
        // // x = flanger.flange(flangeInput, params[ofs+4] * 6000 + 100, params[ofs+5] * 0.98, params[ofs+6] * 0.95f, params[ofs+7]);
        return x;
    }

    void mapParameters(std::vector<float> &newparams) {
        for(size_t i=0; i < params.size(); i++) {
            unsmoothParams[i] = newparams[i];
        }
        filt.set(maxiBiquad::filterTypes::HIGHPASS, unsmoothParams[17] * 5000.f, 2, 1);


        // PERIODIC_DEBUG(6000,
            // printf("1-----\n");
            // union {float f; uint32_t i;} pu;
            // for(size_t i=0; i < newparams.size(); i++) {
            //     pu.f = newparams[i];
                
            //     printf("%x\t", pu.i);
            // }
            // printf("\n");
            // printf("2-----\n");
            // for(size_t i=0; i < params.size(); i++) {
            //     pu.f = params[i];
                
            //     printf("%x\t", pu.i);
            // }
            // printf("\n");
        // )
    }

private:
    maxiNonlinearity distortion;
    maxiDelayline<15000> dl;
    maxiFlanger<15000> flanger;
    maxiOsc sinosc;
    maxiBiquad filt;

    matrixMixer<NFX> mmix;

    std::array<float, NFX> fxInputs;
    std::array<float, NFX> fxOutputs;

    std::vector<float> unsmoothParams, params;
    OnePoleSmoother<kN_synthparams> smoother_;

};

