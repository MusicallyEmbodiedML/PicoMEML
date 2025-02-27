#include "src/PicoDefs.hpp"
#include "src/audio/AudioDriver.hpp"
#include "src/interface/ButtonsPots.hpp"
#include "src/common/common_defs.h"
#include "src/interface/MEMLInterface.hpp"
#include "src/interface/mlp_task.hpp"
#include "src/audio/AnalysisParams.hpp"
#include "src/interface/PIOUART.hpp"

#include "src/audio/app/Passthrough.hpp"

#include <cstdint>
#include <vector>
#include <memory>

#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "hardware/structs/rosc.h"


// Core 0->1 comms
const size_t kSharedMemSize = 1;
volatile int32_t gSharedMem[kSharedMemSize] { -1 };
mutex_t mutex_0to1;
static queue_t queue_audioparam;
static queue_t queue_interface_pulse;
static queue_t queue_interface_midi;
static volatile bool flag_init_0 = false;
static volatile bool flag_init_1 = false;

const bool waitForSerialOnStart = false;

// Global app state
ts_app_state gAppState = {
    .n_iterations = 500,
    .last_error = 0.0f,
    .exploration_range = 0.4f,
    .app_id = app_id_euclidean,
    .current_dataset = 0,
    .current_model = 0,
    .current_nn_mode = mode_inference,
    .current_expl_mode = expl_mode_pretrain
};
bool gTriggerParamUpdate = false;
std::shared_ptr<AudioAppBase> gAudioApp;

// Global objects
MEMLInterface meml_interface(
    &queue_audioparam,
    &queue_interface_pulse,
    &queue_interface_midi,
    nullptr,
    kNInputParams,
    14  // FIXME this should be linked to audio app!
);

uint32_t pico_get_random_bits(int num_bits) {
    uint32_t random_value = 0;

    // Read RANDOMBIT num_bits times
    for (int i = 0; i < num_bits; i++) {
        // Shift the random value left by 1 and add the RANDOMBIT
        random_value = (random_value << 1) | (rosc_hw->randombit & 0x1);
    }

    return random_value;
}


void setup() {


    //wait for serial
    if (waitForSerialOnStart){
        while(!Serial) {;}
    }
    uint32_t seed = pico_get_random_bits(32);
    // Print the generated seed
    //Serial.printf("Generated Random Seed: %u\n", seed);
    // Seed the standard PRNG
    srand(seed);

    // Audio app init
    gAudioApp = std::make_unique<Passthrough>();
    gAudioApp->Setup(kSampleRate);

    // Set up the mutex;
    AnalysisParamsSetup(gAudioApp->GetAudioParamsSize());

    // AUDIO routine setup
    if (!AudioDriver::Setup()) {
        Serial.println("setup - I2S init failed!");
    }
    // I2S callback is last
    AudioDriver::SetAudioApp(gAudioApp);
    // Wait for init sync
    Serial.println("Audio running");
    flag_init_0 = true;


    while (!flag_init_1) {
        // Wait for other core
    };
}

void AUDIO_FUNC(loop)() {

    {
        // Audio parameter queue receiver:
        // From interface/mlp_task.cpp and interface/MEMLInterface.cpp
        // on core 1
        std::vector<float> audio_params(gAudioApp->GetAudioParamsSize());
        if (queue_try_remove(&queue_audioparam, audio_params.data())) {
            //Serial.println(".");
            gAudioApp->UpdateControlParams(audio_params.data());
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

std::unique_ptr<PIOUART> pio_uart;

void setup1() {
    // Init serial and signal other core
    if (waitForSerialOnStart){
        while(!Serial) {;}
    }
    // devmidi = make_shared<MIDIDevice>();
    Serial.println("Core 1 Start");
    // Core INTERFACE routine setup
    queue_init(&queue_audioparam, sizeof(float)*kN_synthparams, 1);
    queue_init(&queue_interface_pulse, sizeof(float), 1);
    queue_init(&queue_interface_midi, sizeof(ts_midi_note), 1);
    // GPIO/ADC setup
    ButtonsPots::Setup(true);
    for (auto &out_pin : { led_Training, led_MIDI }) {
        pinMode(out_pin, OUTPUT);
    }
    Serial.println("Input Pins Set");
    // MLP setup
    mlp_init(&queue_audioparam,
             kNInputParams,
             kN_synthparams,
             1);
            //  ,
            //  devmidi);
    // PIO UART
    pio_uart = std::make_unique<PIOUART>();
    // Report how many input parameters are there
    Serial.printf("Input params: %d (Joystick: %d, UART: %d).\n",
            kNInputParams, kNJoystickParams, kNExtraSensors);

    // Wait for init sync
    flag_init_1 = true;
    while (!flag_init_0) {
        // Wait for other core
    };
}

void loop1() {
    // Read ADC
    ButtonsPots::Process();
    // Read PIO UART
    pio_uart->Poll();
    // Perform parameter update if needed
    if (gTriggerParamUpdate) {
        meml_interface.UpdatePots();
        gTriggerParamUpdate = false;
    }

    static constexpr uint32_t period_ms = 10;
    // Pulse
#if 1
    // MIDIMessage msg = devmidi->Read();
    // if (msg.received) {
      // Serial.println(msg.msgType);
      // if (msg.msgType == midi::MidiType::NoteOn) {
      //   Serial.println("note");
      // };
    // }
    static constexpr float pulse_every_s = 1;
    static constexpr float count_wraparound = (1000.f * pulse_every_s)
            / static_cast<float>(period_ms);
    static volatile float counter = 0;
    counter++;
    if (counter >= count_wraparound) {
        counter = 0;
        //Serial.println(".");
        std::vector<float> params;
        AnalysisParamsRead(params);
        Serial.println(params[0]);
    }
#endif

    delay(period_ms);
}
