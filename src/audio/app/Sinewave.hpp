#ifndef __SINEWAVE_HPP__
#define __SINEWAVE_HPP__

#include "../AudioAppBase.hpp"
#include <Arduino.h>
#include "../../utils/Random.hpp"
#include "../../synth/maximilian.h"


class Passthrough : public AudioAppBase {

 public:

    size_t GetAudioParamsSize(void) { return kAudioParamsSize * sizeof(float); }
    size_t GetControlParamsSize(void) { return kControlParamsSize * sizeof(float); }

    void Setup(float sample_rate)
    {
        maxiSettings::setup(sample_rate, kNChannels, kBufferSize);
    }

    void Process(float input[][kNChannels],
                 float output[][kNChannels],
                 size_t n_samples) {

        ProcessSampleBySample(input, output, n_samples,
            [&] (float in[2], float out[2]) {
                out[0] = out[1] = osc_.sinebuf(1000);
            });
    }

    void UpdateControlParams(const char *params_mem) {
        float *params_float = reinterpret_cast<float*>(params_mem);
        Serial.print("Received ");
        Serial.println(*params_float);
    }

    void SetAudioParams(char *params_mem) {
        float *params_float = reinterpret_cast<float*>(params_mem);
        *params_float = Random::FloatUniform();
    }

 private:

    static const size_t kAudioParamsSize = 1;
    static const size_t kControlParamsSize = 1;

    maxiOsc osc_;
};


#endif  // __SINEWAVE_HPP__
