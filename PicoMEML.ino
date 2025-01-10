#include "src/PicoDefs.hpp"
#include "src/audio/AudioDriver.hpp"
#include "src/audio/AudioApp.hpp"
#include "src/interface/ButtonsPots.hpp"
#include "src/common/common_defs.h"
#include "src/synth/FMSynth.hpp"
#include "src/interface/MEMLInterface.hpp"
#include "src/interface/mlp_task.hpp"
#include "src/web/WebInterface.hpp"

#include <cstdint>
#include <vector>
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

static queue_t queue_audioparam;
static queue_t queue_interface_pulse;
static queue_t queue_interface_midi;
static volatile bool flag_init_0 = false;
static volatile bool flag_init_1 = false;

const bool waitForSerialOnStart = true;

uint32_t get_random_bits(int num_bits) {
    uint32_t random_value = 0;

    // Read RANDOMBIT num_bits times
    for (int i = 0; i < num_bits; i++) {
        // Shift the random value left by 1 and add the RANDOMBIT
        random_value = (random_value << 1) | (rosc_hw->randombit & 0x1);
    }

    return random_value;
}

// Global app state
ts_app_state gAppState = {
    .n_iterations = 500,
    .last_error = 0.0f,
    .exploration_range = 0.0f,
    .app_id = app_id_fmsynth,
    .current_dataset = 0,
    .current_model = 0,
    .current_nn_mode = mode_inference,
    .current_expl_mode = expl_mode_nnweights
};

// Global objects
MEMLInterface meml_interface(
    &queue_audioparam,
    &queue_interface_pulse,
    &queue_interface_midi,
    &FMSynth::GenParams,
    kN_synthparams
);


void setup() {


    //wait for serial
    if (waitForSerialOnStart){
        while(!Serial) {;}
    }
    uint32_t seed = get_random_bits(32);
    // Print the generated seed
    //Serial.printf("Generated Random Seed: %u\n", seed);
    // Seed the standard PRNG
    srand(seed);

    // AUDIO routine setup
    if (!AudioDriver_Output::Setup()) {
        Serial.println("setup - I2S init failed!");
    }
    AudioAppSetup();
    // I2S callback is last
    AudioDriver_Output::SetCallback(&AudioAppProcess);
    // Wait for init sync
    flag_init_0 = true;
    while (!flag_init_1) {
        // Wait for other core
    };
}

void AUDIO_FUNC(loop)() {

    {
        std::vector<float> audio_params(kN_synthparams);
        if (queue_try_remove(&queue_audioparam, audio_params.data())) {
            Serial.println("A- Audio params received.");
            AudioAppSetParams(audio_params);
        }
    }

    {
        float pulse;
        if (queue_try_remove(&queue_interface_pulse, &pulse)) {
            Serial.println("A- Pulse received.");
        }
    }

    {
        ts_midi_note midi;
        if (queue_try_remove(&queue_interface_midi, &midi)) {
            Serial.println("A- MIDI received.");
        }
    }
}


static bool _buttonsTimerCallback(struct repeating_timer *);


void setup1() {
    // Init serial and signal other core
    if (waitForSerialOnStart){
        while(!Serial) {;}
    }

    // Core INTERFACE routine setup
    queue_init(&queue_audioparam, sizeof(float)*kN_synthparams, 1);
    queue_init(&queue_interface_pulse, sizeof(float), 1);
    queue_init(&queue_interface_midi, sizeof(ts_midi_note), 1);
    // GPIO/ADC setup
    ButtonsPots::Setup(true);
    // MLP setup
    mlp_init(&queue_audioparam, kN_synthparams);

    // Wait for init sync
    flag_init_1 = true;
    while (!flag_init_0) {
        // Wait for other core
    };

    // Set up timer for buttons
    static struct repeating_timer adc_timer;
    static const unsigned int adc_period_ms = 1;
    add_repeating_timer_ms(
        adc_period_ms,
        _buttonsTimerCallback,
        nullptr,
        &adc_timer
    );

    // Start web interface
    WebInterfaceSetup();
    WebInterfaceRun();
}

bool _buttonsTimerCallback(__unused struct repeating_timer *) {
    // Read ADC
    ButtonsPots::Process();

    return true;
}

void loop1() {
    static constexpr uint32_t period_ms = 1;

    // Pulse
#if 1
    static constexpr float pulse_every_s = 1;
    static constexpr float count_wraparound = (1000.f * pulse_every_s)
            / static_cast<float>(period_ms);
    static volatile float counter = 0;
    counter++;
    if (counter >= count_wraparound) {
        counter = 0;
        Serial.println(".");
    }
#endif

    delay(period_ms);
}
