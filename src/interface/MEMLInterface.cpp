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
                             size_t nn_output_size) :
        joystick_inference_(true),
        joystick_current_({ { 0.5, 0.5, 0.5 } }),
        interface_fmsynth_(interface_fmsynth),
        interface_midi_(interface_midi),
        gen_params_fn_ptr_(gen_params_fn_ptr),
        nn_output_size_(nn_output_size),
        draw_speed_(1.f),
        midi_on_(false),
        pulse_on_(false)
{
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
            mlp_inference_nochannel(joystick_current_.as_struct);
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

void MEMLInterface::SetPot(te_joystick_pot pot_n, num_t value)
{
    // Update state of joystick
    if (value < 0) {
        value = 0;
    } else if (value > 1.0) {
        value = 1.0;
    }
    joystick_current_.as_array[pot_n] = value;
}

void MEMLInterface::UpdatePots()
{
    // If inference, run inference here
    if (joystick_inference_) {
        mlp_inference_nochannel(joystick_current_.as_struct);
    }
}

void MEMLInterface::SetPulse(int32_t pulse)
{
    if (pulse_on_) {
        queue_try_add(interface_pulse_, &pulse);
    }
}

void MEMLInterface::SetToggleButton(te_button_idx button_n, int8_t state)
{
    switch(button_n) {

        case toggle_training: {
            if (state == mode_inference && gAppState.current_nn_mode == mode_training) {
                mlp_train();
                mlp_inference_nochannel(joystick_current_.as_struct);
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
#if 0  // Legacy randomiser-randomise the parameters directly
                // Generate random params
                std::vector<float> rand_params(nn_output_size_);
                if (gen_params_fn_ptr_) {
                    gen_params_fn_ptr_(rand_params);
                } else {
                    Serial.println("INTF- Param randomiser is null");
                    break;
                }

                // Send them down to fmsynth
                queue_try_add(
                    interface_fmsynth_,
                    reinterpret_cast<void *>(rand_params.data())
                );

                // Also save them in an intermediate space
                current_fmsynth_params_ = std::move(rand_params);
                Serial.println("INTF- Random params");
#else
                mlp_draw(draw_speed_);
                mlp_inference_nochannel(joystick_current_.as_struct);
#endif
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
                        std::vector<num_t> input{
                            joystick_current_.as_struct.potX,
                            joystick_current_.as_struct.potY,
                            joystick_current_.as_struct.potRotate,
                            1.f  // bias
                        };
                        mlp_add_data_point_tdnn(
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
            mlp_inference_nochannel(joystick_current_.as_struct);
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
            mlp_inference_nochannel(joystick_current_.as_struct);
        } break;

        default: {}
    }
}

void MEMLInterface::SendMIDI(ts_midi_note midi_note)
{
    if (midi_on_) {
        queue_try_add(
            interface_midi_,
            reinterpret_cast<void *>(&midi_note)
        );
    }
}
