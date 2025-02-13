#ifndef __MEML_INTERFACE_HPP__
#define __MEML_INTERFACE_HPP__


#include "../common/common_defs.h"
#include <cstdint>
#include <vector>
#include "pico/util/queue.h"

class MEMLInterface {
 public:

    /** Function pointer type for generating random parameters
    (legacy, deprecated) */
    using GenParamsFn_ptr_t = void (*)(std::vector<float>&);

    /**
     * @brief Constructor connecting all the inter-core queues
     * and the function pointer generating the random params.
     * @param interface_fmsynth Queue to pass the parameter update
     * to the audio app.
     * @param interface_midi Queue to pass MIDI information down
     * to the audio app.
     * @param interface_pulse Queue to pass pulse (period, bar)
     * information to the audio app (legacy, for Euclidean seq).
     * @param gen_params_fn_ptr Function pointer that generates
     * random parameters of the right range and size for the audio
     * app (legacy, deprecated, pass nullptr).
     * @param nn_input_size Input size to the network
     * @param nn_output_size Output size of the network
     */
    MEMLInterface(queue_t *interface_fmsynth,
                  queue_t *interface_midi,
                  queue_t *interface_pulse,
                  GenParamsFn_ptr_t gen_params_fn_ptr,
                  size_t nn_input_size,
                  size_t nn_output_size);
    /**
     * @brief Trigger a pot-set interface change.
     * @param pot_n Pot number.
     * @param value Pot value (0-1).
     */
    void SetPot(size_t pot_n, num_t value);
    void UpdatePots(void);
    void SetToggleButton(te_button_idx button_n, int8_t state);
    void SetSlider(te_slider_idx idx, num_t value);
    void SendMIDI(ts_midi_note midi_note);
    void EnableMIDI(bool midi_on = true);
    void EnablePulse(bool pulse_on = true);
    void SetPulse(int32_t pulse);

 protected:

    // States
    bool joystick_inference_;
    std::vector<float> joystick_current_;
    std::vector<float> current_fmsynth_params_;
    // Channels for outside comms
    queue_t *interface_fmsynth_;
    queue_t *interface_midi_;
    queue_t *interface_pulse_;

    GenParamsFn_ptr_t gen_params_fn_ptr_;

    const size_t nn_input_size_;
    const size_t nn_output_size_;
    num_t draw_speed_;
    bool midi_on_;
    bool pulse_on_;
};


#endif  // __MEML_INTERFACE_HPP__
