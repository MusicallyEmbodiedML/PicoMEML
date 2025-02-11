#pragma once

//#define NDEBUG
#include <cassert>
#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <cmath>
#include "maximilian.h"
#include "OnePoleSmoother.hpp"
#include "../audio/AudioDriver.hpp"

#define PERIODIC_DEBUG(COUNT, FUNC) \
        static size_t ct=0; \
        if (ct++ % COUNT == 0) { \
            FUNC         \
        }

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

    static constexpr size_t kNParams = N*N;

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


class FXDelayer {
public:

    // Sizes (for vector allocations)
    static const size_t NFX = 4;
    static constexpr size_t kNMatrixParams = matrixMixer<NFX>::kNParams;
    struct ts_params {
        float matrix[kNMatrixParams];
        float feedbacks[NFX];  // Feedback params
        float delay[NFX];  // One parameter only (feedback)
    };
    static constexpr size_t kNParams = sizeof(ts_params) / sizeof(float);
    // Voicing
    static constexpr size_t kDLLengths[NFX] {
        static_cast<size_t>(static_cast<float>(kSampleRate) * 0.0651),
        static_cast<size_t>(static_cast<float>(kSampleRate) * 0.1538),
        static_cast<size_t>(static_cast<float>(kSampleRate) * 0.3164),
        static_cast<size_t>(static_cast<float>(kSampleRate) * 0.9103)
    };
    static constexpr float feedbackScaling = 0;

    FXDelayer(size_t sample_rate): smoother_(100.f, sample_rate) {
        maxiSettings::setup(sample_rate, 1, 16);
        mmix.setDirectFeedbackScaling(feedbackScaling);
        std::fill(unsmoothParams.begin(), unsmoothParams.end(), 0);
        std::fill(params.v, params.v + kNParams, 0);
    }

    float play(float x) {
        smoother_.Process(unsmoothParams.data(), params.v);

        std::vector<float> m_param_vec(
            params.s.matrix,
            params.s.matrix+matrixMixer<NFX>::kNParams);
        mmix.set(m_param_vec);
        for (size_t n = 0; n < NFX; n++) {
            fxInputs[n] = x * params.s.feedbacks[n] * params.s.feedbacks[n];
        }

        size_t n = 0;  // DL0
        float delayInput = (mmix.calculateMix(fxOutputs, n) + fxInputs[n]) * 0.5;
        fxOutputs[n] = dl0.play(delayInput, kDLLengths[n]-1, params.s.delay[n] * 0.97);
        n = 1;  // DL1
        delayInput = (mmix.calculateMix(fxOutputs, n) + fxInputs[n]) * 0.5;
        fxOutputs[n] = dl1.play(delayInput, kDLLengths[n]-1, params.s.delay[n] * 0.97);
#if 0
        n = 2;  // DL2
        delayInput = (mmix.calculateMix(fxOutputs, n) + fxInputs[n]) * 0.5;
        fxOutputs[n] = dl2.play(delayInput, kDLLengths[n]-1, params.s.delay[n] * 0.97);
        n = 3;  // DL3
        delayInput = (mmix.calculateMix(fxOutputs, n) + fxInputs[n]) * 0.5;
        fxOutputs[n] = dl3.play(delayInput, kDLLengths[n]-1, params.s.delay[n] * 0.97);

        x = std::accumulate(fxOutputs.begin(), fxOutputs.end(), x);
#else
        
        x = x + fxOutputs[0] + fxOutputs[1];
#endif

        // Soft clip
        return std::tanh(x);
        return x;
    }

    void mapParameters(std::vector<float> &newparams) {
        for(size_t i=0; i < kNParams; i++) {
            unsmoothParams[i] = newparams[i];
        }
    }

    static void GenParams(std::vector<float> &param_vector)
    {
        float rand_scale = 1.f / static_cast<float>(RAND_MAX);

        for(size_t i=0; i < kNParams; i++) {
            param_vector[i] = std::rand() * rand_scale;
        }
    }

private:
    maxiDelayline<kDLLengths[0]> dl0;
    maxiDelayline<kDLLengths[1]> dl1;
    maxiDelayline<kDLLengths[2]> dl2;
    maxiDelayline<kDLLengths[3]> dl3;

    matrixMixer<NFX> mmix;

    std::array<float, NFX> fxInputs;
    std::array<float, NFX> fxOutputs;

    std::array<float, kNParams> unsmoothParams;
    union {
        float v[kNParams];
        ts_params s;
    } params;
    OnePoleSmoother<kNParams> smoother_;

};


const size_t kN_synthparams = FXDelayer::kNParams;
