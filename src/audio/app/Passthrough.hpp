#ifndef __PASSTHROUGH_HPP__
#define __PASSTHROUGH_HPP__

#include "../AudioAppBase.hpp"
#include <Arduino.h>
#include "../../utils/Random.hpp"


class Passthrough : public AudioAppBase {

 public:

    size_t GetAudioParamsSize(void) { return kAudioParamsSize * sizeof(float); }
    size_t GetControlParamsSize(void) { return kControlParamsSize * sizeof(float); }

    void Setup(float sample_rate) {}

    void Process(float input[][kNChannels],
                 float output[][kNChannels],
                 size_t n_samples) {
        CopyBuffer(input, output, n_samples);
    }

    void UpdateControlParams(const char *params_mem) {
        const float *params_float = reinterpret_cast<const float*>(params_mem);
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
};


#endif  // __PASSTHROUGH_HPP__
