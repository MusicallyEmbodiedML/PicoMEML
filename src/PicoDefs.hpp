#ifndef __PICO_DEFS_HPP__
#define __PICO_DEFS_HPP__

#include "common/common_defs.h"
#include "interface/MEMLInterface.hpp"

// Select which example app to run
#define FM_SYNTH         0  ///< FM Synth (new macro)
#define FX_PROCESSOR     1  ///< FX Processor (new macro)
#define EUCLIDEAN        0


#define AUDIO_FUNC(x)    __not_in_flash_func(x)  ///< Macro to make audio function load from mem
#define AUDIO_MEM    __not_in_flash("audio")  ///< Macro to make variable load from mem


// Enable or disable joystick
#define USE_JOYSTICK         1
// Enable or disable extra sensors via UART
#define USE_SERIAL_ADCS      0
// Set how many joystick params
#define JOYSTICK_PARAMS      3
// Set how many extra sensors
#define SERIAL_ADC_PARAMS    1

const size_t kNJoystickParams = ((USE_JOYSTICK) ? JOYSTICK_PARAMS : 0);
// Set how many extra sensors we want to process/use
const size_t kNExtraSensors = ((USE_SERIAL_ADCS) ? SERIAL_ADC_PARAMS : 0);

const size_t kNInputParams = kNJoystickParams + kNExtraSensors;

/**
 * @brief Pin configuration on the Pi Pico 2
 */
enum PinConfig {
    i2c_sgt5000Data = 0,
    i2c_sgt5000Clk = 1,
    led_Training = 2,
#if !EUCLIDEAN
    uart_MidiTX = 4,
    uart_MidiRX = 5,
#endif
    i2s_pDIN = 6,
    i2s_pDOUT = 7,
    i2s_pBCLK = 8,
    i2s_pWS = 9,
    i2s_pMCLK = 10,
    toggle_SaveData = 13,
    button_Randomise = 14,
    toggle_Training = 15,
    button_ClearData = 16,
    led_MIDI = 21,
    // button_ZoomOut = 18,
    // button_ZoomIn = 19,
    uart_PIORx = 18,
    uart_PIOTx = 19,
    pot_JoystickX = 26,
    pot_JoystickY = 27,
    pot_JoystickZ = 28

#ifdef EUCLIDEAN
    ,
    pulse0=20,
    pulse1=21,
    pulse2=12,
    pulse3=11,
    pulse4=3,
    pulse5=4,
    pulse6=5
#endif    
};


// Global objects
/** Global app state container (define only once). */
extern ts_app_state gAppState;
/** Global MEML interface (define only once). */
extern MEMLInterface meml_interface;

extern bool gTriggerParamUpdate;

#endif  // __PICO_DEFS_HPP__
