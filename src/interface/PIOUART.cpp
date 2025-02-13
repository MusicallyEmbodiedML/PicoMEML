#include "PIOUART.hpp"
#include "../utils/SLIP.hpp"


PIOUART::PIOUART(size_t baud_rate) :
    serial_pio_(uart_PIOTx, uart_PIORx),
    slipBuffer{ 0 },
    spiState(SPISTATES::WAITFOREND),
    spiIdx(0)
{
    pinMode(uart_PIOTx, OUTPUT);
    pinMode(uart_PIORx, INPUT);
    serial_pio_.begin(baud_rate);
}

void PIOUART::Poll()
{
    uint8_t spiByte=0;

    if (serial_pio_.available()) {

        switch(spiState) {
            case SPISTATES::WAITFOREND:
            spiByte = serial_pio_.read();
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
            spiByte = serial_pio_.read();
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
            spiByte = serial_pio_.read();
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
}

void PIOUART::Parse_(spiMessage msg)
{
    Serial.printf("SLIP message: %d - %f\n",
            msg.msg, msg.value);
}
