// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once
/*
; I2S audio bidirectional (input/output) block, both in slave mode.
; Requires external BCK and LRCK, usually from the codec directly.
; This block provides both the output and input components together,
; so should not be used in combination with either of the other output or
; input units on the same I2S bus.
;
; Input pin order: DIN, BCK, LRCK
; Set JMP pin to LRCK.
;
; Clock synchronously with the system clock, or *at least* 4x the usual
; bit clock for a given fs (e.g. for 48kHz 24-bit, clock at at least
; (48000 * 24 * 2 (stereo)) * 4 = 9.216 MHz. Ideally 8x or more.
*/

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// -------------- //
// i2s_bidi_slave //
// -------------- //

#define i2s_bidi_slave_wrap_target 0
#define i2s_bidi_slave_wrap 23

static const uint16_t i2s_bidi_slave_program_instructions[] = {
            //     .wrap_target
    0x20a1, //  0: wait   1 pin, 1                   
    0x4001, //  1: in     pins, 1                    
    0x8080, //  2: pull   noblock                    
    0x8000, //  3: push   noblock                    
    0x2021, //  4: wait   0 pin, 1                   
    0x2022, //  5: wait   0 pin, 2                   
    0x6001, //  6: out    pins, 1                    
    0x20a1, //  7: wait   1 pin, 1                   
    0x4001, //  8: in     pins, 1                    
    0x2021, //  9: wait   0 pin, 1                   
    0x6001, // 10: out    pins, 1                    
    0x00cd, // 11: jmp    pin, 13                    
    0x0007, // 12: jmp    7                          
    0x20a1, // 13: wait   1 pin, 1                   
    0x4001, // 14: in     pins, 1                    
    0x8080, // 15: pull   noblock                    
    0x8000, // 16: push   noblock                    
    0x2021, // 17: wait   0 pin, 1                   
    0x6001, // 18: out    pins, 1                    
    0x20a1, // 19: wait   1 pin, 1                   
    0x4001, // 20: in     pins, 1                    
    0x2021, // 21: wait   0 pin, 1                   
    0x6001, // 22: out    pins, 1                    
    0x00d3, // 23: jmp    pin, 19                    
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program i2s_bidi_slave_program = {
    .instructions = i2s_bidi_slave_program_instructions,
    .length = 24,
    .origin = -1,
};

static inline pio_sm_config i2s_bidi_slave_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + i2s_bidi_slave_wrap_target, offset + i2s_bidi_slave_wrap);
    return c;
}
#endif
