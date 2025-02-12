#include "MIDI.hpp"


MIDIDevice::MIDIDevice(int rx_pin, int tx_pin) :
    mySerial_(rx_pin, tx_pin),
    serialMIDI_(mySerial_),
    MIDI_((Transport &)serialMIDI_),
    midi_in_chan_(1),
    midi_out_chan_(1)
{

    MIDI_.begin(midi_in_chan_);
    // TODO set up note receive callbacks
}


void MIDIDevice::SendCC(size_t cc_index, int8_t val)
{
    MIDI_.sendControlChange(cc_index, val, midi_out_chan_);
}

void MIDIDevice::SendParamsAsCC(std::vector<float> params)
{
    // Free CC params are 102-119: cap at lowest of two
    const size_t kCCStart = 102;
    const size_t kCCStop = 120;
    constexpr size_t kNCCs = kCCStop - kCCStart;
    size_t vec_size = params.size();
    size_t param_size = kNCCs < vec_size ? kNCCs : vec_size;

    for (size_t n = 0; n < param_size; n++) {
        SendCC(n + kCCStart, params[n]);
        //Serial.print(n);
        //Serial.print(" ");
    }
    //Serial.println("");
}

