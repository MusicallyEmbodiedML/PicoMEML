#ifndef __MIDI_HPP__
#define __MIDI_HPP__


#include <MIDI.h>
#include <SoftwareSerial.h>
#include "../PicoDefs.hpp"


class MIDIDevice {
 public:
    MIDIDevice(int rx_pin = uart_MidiRX, int tx_pin = uart_MidiTX);
    void SendCC(size_t cc_index, int8_t val);
    void SendParamsAsCC(std::vector<float>params);

 protected:

    using Transport = MIDI_NAMESPACE::SerialMIDI<SoftwareSerial>;
    SoftwareSerial mySerial_;
    Transport serialMIDI_;
    MIDI_NAMESPACE::MidiInterface<Transport> MIDI_;
    size_t midi_in_chan_;
    size_t midi_out_chan_;
};


extern MIDIDevice gMIDIDevice;

#endif  // __MIDI_HPP__
