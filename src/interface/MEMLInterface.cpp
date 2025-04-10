#include "MEMLInterface.hpp"
#include "../PicoDefs.hpp"

// STL includes
#include <string>
#include <utility>
#include <cstdlib>
#include <cmath>

#include "Arduino.h"

// Internal C++ includes
#include "mlp_task.hpp"


MEMLInterface::MEMLInterface(queue_t *interface_fmsynth,
                             queue_t *interface_pulse,
                             queue_t *interface_midi,
                             GenParamsFn_ptr_t gen_params_fn_ptr,
                             size_t nn_input_size,
                             size_t nn_output_size) :
        joystick_inference_(true),
        joystick_current_(nn_input_size),
        interface_fmsynth_(interface_fmsynth),
        interface_midi_(interface_midi),
        gen_params_fn_ptr_(gen_params_fn_ptr),
        nn_input_size_(nn_input_size),
        nn_output_size_(nn_output_size),
        draw_speed_(1.f),
        midi_on_(false),
        pulse_on_(false)
{
    // Initialise to "middle value"
    for (auto &j : joystick_current_) {
        j = 0.5;
    }
}

void MEMLInterface::EnableMIDI(bool midi_on)
{
    midi_on_ = midi_on;
}

void MEMLInterface::EnablePulse(bool pulse_on)
{
    pulse_on_ = pulse_on;
}

void MEMLInterface::SetSlider(te_slider_idx idx, num_t value)
{
    switch(idx) {
        case slider_randomSpeed: {
            gAppState.exploration_range = value;
            // TODO deprecate those local values!!!
            draw_speed_ = value;
            mlp_set_speed(draw_speed_);
            mlp_inference(joystick_current_);
        } break;

        case slider_nIterations: {
            gAppState.n_iterations = value;
        } break;

        default: {
            Serial.print("INTF- Slider idx ");
            Serial.print(idx);
            Serial.println(" not recognised!");
        }
    }
}

void MEMLInterface::SetPot(size_t pot_n, num_t value)
{
    // Update state of joystick
    if (value < 0) {
        value = 0;
    } else if (value > 1.0) {
        value = 1.0;
    }
    if (pot_n < nn_input_size_) {
        joystick_current_[pot_n] = value;
    } else {
        Serial.print("INTF- Input index ");
        Serial.print(pot_n);
        Serial.println(" out of bounds.");
    }
}

void MEMLInterface::UpdatePots()
{
    // If inference, run inference here
    if (joystick_inference_) {
        mlp_inference(joystick_current_);
    }
}

void MEMLInterface::SetPulse(int32_t pulse)
{
    if (pulse_on_) {
        //queue_try_add(interface_pulse_, &pulse);
    }
}

void MEMLInterface::SetToggleButton(te_button_idx button_n, int8_t state)
{
    switch(button_n) {

        case toggle_training: {
            if (state == mode_inference && gAppState.current_nn_mode == mode_training) {
                mlp_train();
                mlp_inference(joystick_current_);
            }
            gAppState.current_nn_mode = static_cast<te_nn_mode>(state);
            // Set the LED
            digitalWrite(led_Training, gAppState.current_nn_mode);
            std::string dbg_mode(( gAppState.current_nn_mode == mode_training ) ? "training" : "inference");
            Serial.print("INTF- Mode: ");
            Serial.println(dbg_mode.c_str());

        } break;

        case button_randomise: {
            if (gAppState.current_nn_mode == mode_training) {
                // Randomise through MLP
                mlp_draw(draw_speed_);
                mlp_inference(joystick_current_);
            }
        } break;

        case toggle_savedata: {
            if (gAppState.current_nn_mode == mode_training) {
                if (state) {  // Button pressed/toggle on
                    joystick_inference_ = false;
                    Serial.println("INTF- Move the joystick to where you want it...");
                } else {  // Button released/toggle off
                    if (mlp_stored_output.size() > 0) {
                        // Save data point
                        std::vector<num_t> input(joystick_current_);
                        mlp_add_data_point(
                            input, mlp_stored_output
                        );
                        Serial.println("INTF- Saved data point");
                    } else {
                        Serial.println("INTF- Data point skipped");
                    }
                    joystick_inference_ = true;
                }
            }
        } break;

        case button_cleardata: {
            mlp_clear_data();
        } break;

        case button_clearmodel: {
            mlp_clear_model();
            mlp_inference(joystick_current_);
        } break;

        case toggle_explmode: {
            te_expl_mode expl_mode = static_cast<te_expl_mode>(state);
            std::string dbg_expl_mode;
            switch(expl_mode) {
                case expl_mode_nnweights: {
                    dbg_expl_mode = "nnweights";
                } break;
                case expl_mode_pretrain: {
                    dbg_expl_mode = "pretrain";
                } break;
                case expl_mode_zoom: {
                    dbg_expl_mode = "zoom";
                } break;
                default: {}
            }
            Serial.print("INTF- Exploration_mode '");
            Serial.print(dbg_expl_mode.c_str());
            Serial.println("'.");
            mlp_set_expl_mode(expl_mode);
        } break;

        case toggle_dataset: {
            mlp_set_dataset_idx(state);
        } break;

        case toggle_model: {
            mlp_set_model_idx(state);
            mlp_inference(joystick_current_);
        } break;

        default: {}
    }
}

void MEMLInterface::SendMIDI(ts_midi_note midi_note)
{
    if (midi_on_) {
        // queue_try_add(
        //     interface_midi_,
        //     reinterpret_cast<void *>(&midi_note)
        // );
    }
}
