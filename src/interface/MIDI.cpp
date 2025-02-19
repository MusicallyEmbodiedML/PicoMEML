#include "MIDI.hpp"
// #include <Arduino.h>

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
        SendCC(n + kCCStart, static_cast<int8_t>(params[n] * 127));
        // Serial.print(n);
        // Serial.print(" ");
    }
    //Serial.println("");
    // SendCC(1, static_cast<int8_t>(params[0] * 127));
}

MIDIMessage MIDIDevice::Read() {
    MIDIMessage msg;
    msg.received = false;
    if (MIDI_.read()) {        
            // Get the type of message received
            midi::MidiType messageType = MIDI_.getType();
    
            // Get the channel, data1 (note number or control), and data2 (velocity or value)
            uint8_t channel = MIDI_.getChannel();
            uint8_t data1 = MIDI_.getData1();
            uint8_t data2 = MIDI_.getData2();
    
            Serial.print("MIDI Message: ");
            Serial.print("Type: "); Serial.print(messageType);
            Serial.print(", Channel: "); Serial.print(channel);
            Serial.print(", Data1: "); Serial.print(data1);
            Serial.print(", Data2: "); Serial.println(data2);
                // Serial.printf("midi read %u %u %u\n", MIDI_.getType(), MIDI_.getData1(), MIDI_.getData2());
        // msg.received=true;
        // msg.msgType = MIDI_.getType();
        // msg.data1 = MIDI_.getData1();
        // msg.data2 = MIDI_.getData2();
    }
    return msg;
}