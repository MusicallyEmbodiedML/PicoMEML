#ifndef __BUTTONS_POTS_HPP__
#define __BUTTONS_POTS_HPP__

#include "../PicoDefs.hpp"
#include "../common/common_defs.h"

#include <cstdint>
#include <cstring>


/**
 * @brief Manage interface in/out on the pico.
 */
class ButtonsPots {

 public:

    /**
     * @brief Set up the button and ADC interface.
     * @param button_isr GPIO callbacks are called on an ISR (good idea)
     */
    static void Setup(bool button_isr = false);
    /**
     * @brief Process the ADCs and (optionally) buttons.
     * Called in the loop1() loop.
     */
    static void Process(void);

    /** Pin indexes that are either buttons or toggles.
    Add new buttons to this array to have them initialised
    automatically. */
    static constexpr enum PinConfig Buttons[] {
        toggle_SaveData,
        button_Randomise,
        toggle_Training,
        button_ClearData,
        //button_ZoomOut,
        //button_ZoomIn
    };
    /** Number of pins that are buttons */
    static constexpr size_t kNButtons = sizeof(Buttons)/sizeof(PinConfig);
    /** Pin indexes that are ADC pots */
    static constexpr enum PinConfig Pots[] {
        pot_JoystickX,
        pot_JoystickY,
        pot_JoystickZ
    };
    /** Number of pins that are ADC */
    static constexpr size_t kNPots = sizeof(Pots)/sizeof(PinConfig);

 protected:

    static void ButtonISR(void);
    static void ProcessButton_(const PinConfig pin_n,
                               const size_t n,
                               bool button_current_state);
};


#endif  // __BUTTONS_POTS_HPP__
