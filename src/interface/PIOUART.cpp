#include "PIOUART.hpp"
#include "../utils/SLIP.hpp"
#include "MEMLInterface.hpp"


PIOUART::PIOUART(size_t baud_rate) :
    serial_pio_(uart_PIOTx, uart_PIORx),
    slipBuffer{ 0 },
    filters_(),
    value_states_{ 0 },
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
    uint8_t spiByte=0;

    while (serial_pio_.available()) {

        //Serial.print(".");
        spiByte = serial_pio_.read();
        //Serial.print(spiByte);
        //Serial.print("->");
#if 0
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

        Serial.print(spiState);
        Serial.print(" ");
#endif

    //Serial.print(spiByte);
    //Serial.print(" ");
    spiMessage decodeMsg { 0, static_cast<float>(spiByte)/255.f };
    Parse_(decodeMsg);    

    }  // serial_pio_.available()

    if (spiIdx >= static_cast<int>(kSlipBufferSize_)) {
        Serial.println("PIOUART- Buffer overrun!!!");
    }

}

void PIOUART::Parse_(spiMessage msg)
{
    static const float kEventThresh = 0.01;

    if (msg.msg < value_states_.size()) {
        //Serial.print(msg.value);
        //Serial.print(" ");
        float filtered_value = filters_[msg.msg].process(msg.value);
        //Serial.print(filtered_value);
        //Serial.print(" ");
        float prev_value = value_states_[msg.msg];
        if (std::abs(filtered_value - prev_value) > kEventThresh) {
            meml_interface.SetPot(msg.msg, filtered_value);
            //Serial.print(filtered_value);
            //Serial.print(" ");
            meml_interface.UpdatePots();
        }
        Serial.print("Low:0.00,High:1.00,Value:");
        Serial.print(msg.value);
        Serial.print(",FilteredValue:");
        Serial.println(filtered_value);
    }
}
