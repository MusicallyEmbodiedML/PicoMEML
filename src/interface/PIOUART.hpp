#ifndef __PIOUART_HPP__
#define __PIOUART_HPP__


#include "../PicoDefs.hpp"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "../utils/MedianFilter.h"

#include <array>


class PIOUART {

 public:
    PIOUART(size_t baud_rate = 115200);
    void Poll();

 protected:
    static const size_t kSlipBufferSize_ = 64;
    SerialPIO serial_pio_;
    uint8_t slipBuffer[kSlipBufferSize_];
    std::array<MedianFilter<float>, kNExtraSensors> filters_;
    std::array<float, kNExtraSensors> value_states_;

    struct spiMessage {
    uint8_t msg;
    float value;
    };
    enum SPISTATES {WAITFOREND, ENDORBYTES, READBYTES};
    SPISTATES  spiState;
    int spiIdx;

    void Parse_(spiMessage msg);
};


#endif  // __PIOUART_HPP__
