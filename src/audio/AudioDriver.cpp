#include "AudioDriver.hpp"
#include "../PicoDefs.hpp"
#include <Arduino.h>
#include "Wire.h"

#include "control_sgtl5000.h"
#include "i2s_pio/i2s.h"
#include "../synth/maximilian.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

const int AUDIO_MEM sampleRate = kSampleRate; // minimum for many i2s DACs
const int AUDIO_MEM bitsPerSample = 32;

const float AUDIO_MEM amplitude = 1 << (bitsPerSample - 2); // amplitude of square wave = 1/2 of maximum
const float AUDIO_MEM neg_amplitude = -amplitude; // amplitude of square wave = 1/2 of maximum
const float AUDIO_MEM one_over_amplitude = 1.f / amplitude;

int32_t sample = amplitude; // current sample value

AudioControlSGTL5000 codecCtl;

static std::shared_ptr<AudioAppBase> AUDIO_MEM audio_app_ptr_ = nullptr;


static __attribute__((aligned(8))) pio_i2s i2s;

inline float AUDIO_FUNC(_scale_and_saturate)(float x) {
    x *= amplitude;
    if (x > amplitude) {
        x = amplitude;
    } else if (x < neg_amplitude) {
        x = neg_amplitude;
    }
    return x;
}

inline float AUDIO_FUNC(_scale_down)(float x) {
    return x * one_over_amplitude;
}


static void AUDIO_FUNC(process_audio)(const int32_t* input, int32_t* output, size_t num_frames) {
    // Timing start
    auto ts = micros();
    const float *input_as_float = reinterpret_cast<float*>(input);
    const float *output_as_float = reinterpret_cast<float*>(output);

    // Convert to float
    for (size_t i = 0; i < num_frames; i++) {
        input_as_float[input[i*2]] = _scale_down(static_cast<float>(input[i*2]));
        input_as_float[input[i*2+1]] = _scale_down(static_cast<float>(input[i*2+1]));
    }

    if (audio_app_ptr_) {
        // Run audio app
        audio_app_ptr_->Process(input_as_float, output_as_float, num_frames);
    } else {
        // Clean passthrough
        for (size_t n = 0; n < num_frames * kNChannels; n++) {
            output_as_float[n] = input_as_float[n];
        }
    }

    // Convert back to int
    for (size_t i = 0; i < num_frames; i++) {
        output[i*2] = static_cast<int32_t>(_scale_and_saturate(output_as_float[i*2]));
        output[i*2+1] = static_cast<int32_t>(_scale_and_saturate(output_as_float[i*2+1]));
    }
    // Timing end
    auto elapsed = micros() - ts;
    static constexpr float quantumLength = 1.f/
            ((static_cast<float>(kBufferSize)/static_cast<float>(kSampleRate))
            * 1000000.f);
    float dspload = elapsed * quantumLength;

    // Report DSP overload if needed
    static volatile bool dsp_overload = false;
    if (dspload > 0.95 and !dsp_overload) {
        dsp_overload = true;
    } else if (dspload < 0.9) {
        dsp_overload = false;
    }
}

static void __isr dma_i2s_in_handler(void) {
    /* We're double buffering using chained TCBs. By checking which buffer the
     * DMA is currently reading from, we can identify which buffer it has just
     * finished reading (the completion of which has triggered this interrupt).
     */
    if (*(int32_t**)dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr == i2s.input_buffer) {
        // It is inputting to the second buffer so we can overwrite the first
        process_audio(i2s.input_buffer, i2s.output_buffer, AUDIO_BUFFER_FRAMES);
    } else {
        // It is currently inputting the first buffer, so we write to the second
        process_audio(&i2s.input_buffer[STEREO_BUFFER_SIZE], &i2s.output_buffer[STEREO_BUFFER_SIZE], AUDIO_BUFFER_FRAMES);
    }
    dma_hw->ints0 = 1u << i2s.dma_ch_in_data;  // clear the IRQ
}

bool AudioDriver::Setup() {

    audio_app_ptr_ = nullptr;

    maxiSettings::setup(kSampleRate, 2, kBufferSize);

    if (!Wire.setSDA(i2c_sgt5000Data) ||
            !Wire.setSCL(i2c_sgt5000Clk)) {
        Serial.println("AUDIO- Failed to setup I2C with codec!");
    }

    set_sys_clock_khz(132000*2, true);
    // set_sys_clock_khz(129600, true);
    Serial.printf("System Clock: %lu\n", clock_get_hz(clk_sys));

    i2s_config picoI2SConfig {
        kSampleRate, // 48000,
        256,
        bitsPerSample, // 32,
        i2s_pMCLK, // 10,
        i2s_pDIN,  // 6,
        i2s_pDOUT,  // 7,
        i2s_pBCLK, // 8,
        true
    };
    i2s_program_start_synched(pio0, &picoI2SConfig, dma_i2s_in_handler, &i2s);

    // init i2c
    codecCtl.enable();
    codecCtl.volume(0.8);
    codecCtl.inputSelect(AUDIO_INPUT_LINEIN);
    codecCtl.lineInLevel(7);


    return true;
}

void AudioDriver::SetAudioApp(std::shared_ptr<AudioAppBase> audio_app_ptr)
{
    audio_app_ptr_ = audio_app_ptr;
}

