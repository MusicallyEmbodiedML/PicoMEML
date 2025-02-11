#include "MIDI.hpp"


Adafruit_USBD_MIDI usb_midi_;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi_, MIDI);


USBMIDIDevice::USBMIDIDevice()
{
    if (!TinyUSBDevice.isInitialized()) {
        TinyUSBDevice.begin(0);
    }
    usb_midi_.setStringDescriptor("TinyUSB MIDI");

    // Initialize MIDI, and listen to all MIDI channels
    // This will also call usb_midi's begin()
    MIDI.begin(MIDI_CHANNEL_OFF);

    // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }

#if TINYUSB_NEED_POLLING_TASK
    Serial.println("MIDI_USB: tud_task polling required.");
#endif
}


void USBMIDIDevice::MIDISendCC(size_t cc_index, int8_t val)
{
    //MIDI.sendControlChange(BUTTONS[i]->Bvalue, 127, BUTTONS[i]->Bchannel);
}
