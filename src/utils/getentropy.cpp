#include <Arduino.h>
#include <RP2040Support.h>


extern RP2040 rp2040;

extern "C" __attribute__((weak)) int getentropy (void * buffer, size_t how_many) {
    uint8_t* pBuf = (uint8_t*) buffer;
    while(how_many--) {
        uint8_t rand_val = rp2040.hwrand32() % UINT8_MAX;
        *pBuf++ = rand_val;
    }
    return 0; // return "no error". Can also do EFAULT, EIO, ENOSYS
}
