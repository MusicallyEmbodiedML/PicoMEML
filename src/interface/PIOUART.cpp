#include "PIOUART.hpp"
#include "../utils/SLIP.hpp"
#include "MEMLInterface.hpp"


PIOUART::PIOUART(size_t baud_rate) :
    serial_pio_(SerialPIO::NOPIN, uart_PIORx),
    slipBuffer{ 0 },
    filters_(),
#if USE_SERIAL_ADCS
    value_states_{ 0 },
#else
    value_states_{},
#endif
    spiState(SPISTATES::WAITFOREND),
    spiIdx(0)
{
    pinMode(uart_PIOTx, OUTPUT);
    pinMode(uart_PIORx, INPUT);
    serial_pio_.begin(baud_rate);

    // Put all values in the middle
    for (auto &v : value_states_) {
        v = 0.5f;
    }
}

void PIOUART::Poll()
{
#if USE_SERIAL_ADCS
    uint8_t spiByte = 32;

 
    while (serial_pio_.available()) {
        spiByte = serial_pio_.read();

        switch(spiState) {
            case SPISTATES::WAITFOREND:
            //spiByte = serial_pio_.read();
            if (spiByte != -1) {
                if (spiByte == SLIP::END) {
                    // Serial.println("end");
                    slipBuffer[0] = SLIP::END;
                    spiState = SPISTATES::ENDORBYTES;
                }else{
                  // Serial.println(spiByte);
                }
            }
            break;
            case ENDORBYTES:
            //spiByte = serial_pio_.read();
            if (spiByte != -1) {
                if (spiByte == SLIP::END) {
                    //this is the message start
                    spiIdx = 1;

                }else{
                    slipBuffer[1] = spiByte;
                    spiIdx=2;
                }
                spiState = SPISTATES::READBYTES;
            }
            break;
            case READBYTES:
            //spiByte = serial_pio_.read();
            if (spiByte != -1) {

                slipBuffer[spiIdx++] = spiByte;
                if (spiByte == SLIP::END) {
                spiMessage decodeMsg;
                SLIP::decode(slipBuffer, spiIdx, reinterpret_cast<uint8_t*>(&decodeMsg));

                // Process the decoded message
                Parse_(decodeMsg);

                spiState = SPISTATES::WAITFOREND;
                }
            }
            break;
        }  // switch(spiState)

    }  // serial_pio_.available()

    if (spiIdx >= static_cast<int>(kSlipBufferSize_)) {
        Serial.println("PIOUART- Buffer overrun!!!");
    }
#endif  // USE_SERIAL_ADCS
}

void PIOUART::Parse_(spiMessage msg)
{
    static const float kEventThresh = 0.01;
    static const size_t kObservedChan = 0;

    if (msg.msg < value_states_.size()) {
        float filtered_value = filters_[msg.msg].process(msg.value);
        float prev_value = value_states_[msg.msg];
        if (std::abs(filtered_value - prev_value) > kEventThresh) {
            meml_interface.SetPot(kNJoystickParams + msg.msg, filtered_value);
            // Trigger param update (or NN inference)
            gTriggerParamUpdate = true;
        }
        if (kObservedChan == msg.msg) {
            Serial.print("Low:0.00,High:1.00,Value:");
            Serial.print(msg.value, 8);
            Serial.print(",FilteredValue:");
            Serial.println(filtered_value, 8);
        }
        value_states_[msg.msg] = filtered_value;
    }
}
