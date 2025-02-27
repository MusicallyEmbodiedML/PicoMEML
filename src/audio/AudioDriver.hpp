#ifndef __AUDIO_DRIVER_HPP__
#define __AUDIO_DRIVER_HPP__

#include <cstddef>
#include <memory>
#include <Arduino.h>
#include "AudioAppBase.hpp"


const size_t kBufferSize = 64;
const size_t kSampleRate = 48000;
const size_t kNChannels = AudioAppBase::kNChannels;
constexpr float kSampleRateRcpr = 1.0/kSampleRate;


class AudioDriver {

 public:

    static bool Setup();
    static void SetAudioApp(std::shared_ptr<AudioAppBase> audio_app_ptr);

 private:
    AudioDriver() = delete;
};


#endif  // __AUDIO_DRIVER_HPP__
