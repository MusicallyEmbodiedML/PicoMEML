#ifndef __AUDIO_APP_BASE_HPP__
#define __AUDIO_APP_BASE_HPP__


#include <cstring>


class AudioAppBase {
 public:

    static const size_t kNChannels = 2;

    virtual size_t GetAudioParamsSize() const = 0;
    virtual size_t GetControlParamsSize() const = 0;

    virtual void Setup(float sample_rate) = 0;
    virtual void Process(float input[][kNChannels], float output[][kNChannels], size_t n_samples) = 0;
    virtual void UpdateControlParams(const char *params_mem) = 0;
    virtual void SetAudioParams(char *params_mem) = 0;

 protected:

    // Static helpers
    static inline void CopyBuffer(float input[][kNChannels],
                 float output[][kNChannels],
                 size_t n_samples) {
        std::memcpy(output, input, n_samples*kNChannels*sizeof(float));
    }
    template <typename Fn>
    static inline void ProcessSampleBySample(float input[][kNChannels],
                 float output[][kNChannels],
                 size_t n_samples,
                 Fn fn_ptr) {
        for (unsigned int n=0; n < n_samples; n++) {
            fn_ptr(input[n], output[n]);
        }
    };
};


#endif  // __AUDIO_APP_BASE_HPP__
