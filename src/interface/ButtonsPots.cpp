#include "ButtonsPots.hpp"
#include "../utils/MedianFilter.h"
#include "../common/common_defs.h"

#include <Arduino.h>
#include <map>


static bool button_current_[ButtonsPots::kNButtons];
static bool button_state_[ButtonsPots::kNButtons];
static float pot_current_[ButtonsPots::kNPots];
static float pot_state_[ButtonsPots::kNPots];

MedianFilter<float> adc_filters[ButtonsPots::kNPots];
MedianFilter<bool> button_filters[ButtonsPots::kNButtons];

static bool button_isr_ = false;


void ButtonsPots::Setup(bool button_isr)
{
    button_isr_ = button_isr;
    // Set GPIO buttons as read
    for (auto &gpio_idx : Buttons) {
        pinMode(gpio_idx, INPUT_PULLUP);
        // Set up ISR
        if (button_isr) {
            attachInterrupt(digitalPinToInterrupt(gpio_idx), ButtonISR, CHANGE);
        }
    }
    // Set analogue as ADC
    for (auto &gpio_idx : Pots) {
        pinMode(gpio_idx, INPUT);
    }

    // Reset states
    for (unsigned int n = 0; n < kNButtons; n++) {
        button_state_[n] = true;
        button_current_[n] = true;
    }
    for (unsigned int n = 0; n < kNPots; n++) {
        pot_state_[n] = 0;
        pot_current_[n] = 0;
    }
}


float _sat(float x) {
    x = (x < 0) ? 0 : x;
    return (x > 1.f) ? 1.f : x;
}


void ButtonsPots::Process(void)
{
#if USE_JOYSTICK
    static constexpr float kPotThreshold = 100.f;
    static constexpr float kPotScaling = 1.f / static_cast<float>(1 << 10);

    bool pot_changed = false;
    for (unsigned int n = 0; n < kNPots; n++) {
        pot_current_[n] = adc_filters[n].process(analogRead(Pots[n]));
        if (std::abs(pot_current_[n] - pot_state_[n]) > kPotThreshold) {
            pot_state_[n] = pot_current_[n];
            pot_changed = true;
            // Pot action
            meml_interface.SetPot(static_cast<te_joystick_pot>(n),
                    _sat(pot_state_[n] * kPotScaling));
        }
    }
    if (pot_changed) {
        ts_joystick_read joystick_read {
            _sat(pot_state_[0] * kPotScaling),
            _sat(pot_state_[1] * kPotScaling),
            _sat(pot_state_[2] * kPotScaling),
        };
        //Serial.print("X:");
        //Serial.print(joystick_read.potX);
        //Serial.print(",Y:");
        //Serial.print(joystick_read.potY);
        //Serial.print(",Rotate:");
        //Serial.println(joystick_read.potRotate);
        // Pot action
        gTriggerParamUpdate = true;
    }
#endif  // USE_JOYSTICK

    if (!button_isr_) {
        for (unsigned int n = 0; n < kNButtons; n++) {
            button_current_[n] = digitalRead(Buttons[n]);
            ProcessButton_(Buttons[n], n, button_current_[n]);
        }
    }
}


void ButtonsPots::ProcessButton_(const PinConfig pin_n,
                                 const size_t n,
                                 bool button_current_state) {

    bool button_previous_state = button_state_[n];

    // Action
    if (button_current_state != button_previous_state) {

        // Debounce here
    #if 1
        static constexpr long debounce_ms = 20;
        static unsigned long last_interrupt_time = 0;
        unsigned long interrupt_time = millis();
        long diff = static_cast<long>(interrupt_time) -
                static_cast<long>(last_interrupt_time);
        if (interrupt_time >= last_interrupt_time
                && diff < debounce_ms) {
            last_interrupt_time = interrupt_time;
            return;  // Ignore bounce
        }
        // Serial.print("Interrupt: ");
        // Serial.print(interrupt_time);
        // Serial.print(" - ");
        // Serial.print(last_interrupt_time);
        // Serial.print(" = ");
        // Serial.println(diff);
        last_interrupt_time = interrupt_time;
    #endif

        button_state_[n] = button_current_state;

        Serial.print("GPIO ");
        Serial.print(pin_n);
        Serial.print(" state ");
        Serial.println(button_state_[n]);

        bool state = !button_state_[n];

        // Map pin numbers to correct button indexes
        static const std::map<PinConfig, te_button_idx> pin_to_button {
            { toggle_SaveData, toggle_savedata },
            { button_Randomise, button_randomise },
            { toggle_Training, toggle_training },
            { button_ClearData, button_cleardata }
        };
        auto button_pos = pin_to_button.find(pin_n);
        te_button_idx button_n;
        if (button_pos == pin_to_button.end()) {
            Serial.println("GPIO pin not recognised/configured.");
            return;
        } else {
            button_n = button_pos->second;
        }

        // Trigger action
        switch(pin_n) {
            // Buttons
            case button_Randomise:
            case button_ClearData:
            {
                if (state) {
                    // Trigger button action
                    meml_interface.SetToggleButton(button_n, state);
                }
            } break;
            // Toggles
            case toggle_SaveData:
            case toggle_Training:
            {
                // Trigger toggle action
                meml_interface.SetToggleButton(button_n, state);
            } break;
            default: {}  // Unrecognised pin
        }
    }
}


void ButtonsPots::ButtonISR(void)
{
    for (unsigned int n = 0; n < kNButtons; n++) {
        PinConfig pin_n = Buttons[n];
        bool button_current_state = digitalRead(pin_n);

        ProcessButton_(pin_n, n, button_current_state);

    }
}