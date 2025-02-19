#include "FMSynth.hpp"
#include <cmath>
#include <random>
#include <cstdlib>
#include <vector>

void FMSynth::GenParams(std::vector<float> &param_vector)
{
#if 0
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> dis(0.f, 1.0f);
#else
    float rand_scale = 1.f / static_cast<float>(RAND_MAX);
#endif
    //printf("Calling FMSynth::GenParams\n");

    for(size_t i=0; i < kN_synthparams; i++) {
        param_vector[i] = std::rand() * rand_scale;
        //printf(".");
    }
    //printf("\n");
}

void FMSynth::UpdateParams() {
    op1.UpdateParams();
    op2.UpdateParams();
    op3.UpdateParams();
    op4.UpdateParams();
    
}

FMSynth::FMSynth(float sample_rate) :
    smoother_(100.f, sample_rate),
    envelope_smoother_(10.f, sample_rate),
    note_freq_(0),
    note_amplitude_(0),
    play_note_(false),
    midi_enabled_(true)
{
    // std::srand(0);
    maxiSettings::setup(sample_rate, 1, 16);
    UpdateParams();
    std::vector<float> randParams(kN_synthparams);
    GenParams(randParams);

    mapParameters(randParams);
    
}

void FMSynth::mapParameters(std::vector<float> &params) {
    float *params_ptr = params.data();
    float *dest_ptr = synthparams.data();

    *dest_ptr++ = 20 + ((*(params_ptr) * *(params_ptr)) * 5000);
    ++params_ptr;
    *dest_ptr++ = 20 + ((*(params_ptr) * *(params_ptr)) * 3000);
    ++params_ptr;
    *dest_ptr++ = (*(params_ptr++) * 200);

    *dest_ptr++ = 20 + ((*(params_ptr) * *(params_ptr)) * 5000);
    ++params_ptr;
    *dest_ptr++ = 20 + ((*(params_ptr) * *(params_ptr)) * 5000);
    ++params_ptr;
    *dest_ptr++ = (*(params_ptr++) * 200);

    *dest_ptr++ = (*(params_ptr++) * 200);

    *dest_ptr++ = 20 + ((*(params_ptr) * *(params_ptr)) * 5000);
    ++params_ptr;
    *dest_ptr++ = 20 + ((*(params_ptr) * *(params_ptr)) * 1000);
    ++params_ptr;
    *dest_ptr++ =  (params[9] * 200);

    *dest_ptr++ = 20 + ((*(params_ptr) * *(params_ptr)) * 5000);
    ++params_ptr;
    *dest_ptr++ = 20 + ((*(params_ptr) * *(params_ptr)) * 800);
    ++params_ptr;
    *dest_ptr++ =  (*(params_ptr++) * 100);

    *dest_ptr++ = (*(params_ptr++) * 200);


}

inline float midiNoteToFrequency(int midiNote) {
    // Constants
    constexpr float A440 = 440.0f; // Frequency of A4
    constexpr float SEMITONE_RATIO = 1.059463094359f; // 2^(1/12), the ratio between adjacent semitones

    // Middle C (C4) is MIDI note 60, A4 is MIDI note 69
    int semitoneOffset = midiNote - 69;

    // Calculate the frequency
    return A440 * std::pow(SEMITONE_RATIO, semitoneOffset);
}
size_t sampleIdx=0;
maxiOsc tmposc;
float FMSynth::process()
{
    // Smooth all parameters before using them
    smoother_.Process(synthparams.data(), synthparams_smoothed.data());

    float carrier_1, carrier_2, envelope;

    // Handle MIDI
#if 1
    if (midi_enabled_) {
#else
    if (false) {
#endif
        carrier_1 = note_freq_;
        carrier_2 = carrier_1;
        if (play_note_ == false) {
            // No notes to play
            envelope = 0;
        } else {
            // One note to play!
            envelope = note_amplitude_;
        }
        // Smooth envelope
        float envelope_smoothed;
        envelope_smoother_.Process(&envelope, &envelope_smoothed);
        envelope = envelope_smoothed;
    } else {
        carrier_1 = synthparams_smoothed[0] * 0.2f;
        carrier_2 = synthparams_smoothed[7] * 0.6;
        envelope = 1.0f;
    }

#if 1
    float w = op1.play(carrier_1 +
        (op2.play(synthparams_smoothed[3],synthparams_smoothed[4],synthparams_smoothed[5]) * synthparams_smoothed[6]),
        synthparams_smoothed[1], synthparams_smoothed[2]);

    float w2 = op3.play(carrier_2 +
        (op4.play(synthparams_smoothed[10],synthparams_smoothed[11],synthparams_smoothed[12]) * synthparams_smoothed[13]),
        synthparams_smoothed[8], synthparams_smoothed[9]);

    float y = (w + w2) * envelope;

    // float y = op1.play(500,1, 1);
    // if (sampleIdx++ %1000 == 0) {
    //     Serial.println(y);
    // }

    return std::tanh(y);
#else
    return op1.play(carrier_1, 0, 0) * envelope;
#endif
}

int32_t FMSynth::processInt()
{
    static const float scaling = std::pow(2.f, 31.f) - 1000.f;

    return static_cast<int32_t>(process() * scaling);
}

void FMSynth::EnableMIDI(bool en)
{
    midi_enabled_ = en;
}

void FMSynth::AddMIDINote(ts_midi_note note)
{
    if (midi_enabled_) {
        if (note.velocity > 0) {
            // note_buffer_.push_back(note);
            note_freq_ = midiNoteToFrequency(note.note_number);
            note_amplitude_ = note.velocity;
            play_note_ = true;
        } else {
        // #if 0
        //     note_buffer_.RemoveNote(note);
        // #else
        //     note_buffer_.clear();
        // #endif
            play_note_ = false;
        }
    }
}