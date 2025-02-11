#ifndef __MIDI_HPP__
#define __MIDI_HPP__


#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <cstdint>
#include <cstddef>


class USBMIDIDevice {
 public:
    USBMIDIDevice();
    void MIDISendCC(size_t cc_index, int8_t val);

 protected:
};


#endif  // __MIDI_HPP__
