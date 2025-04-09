#ifndef __DAISY_UART_SEND_HPP__
#define __DAISY_UART_SEND_HPP__


#include <Arduino.h>
#include <SerialPIO.h>


/**
 * @brief Sends float data to the Daisy using SLIP encoding over a PIO-based
 * "software" UART on a single GPIO pin (txPin).
 */
class DaisyUARTSend
{
public:
    /**
     * @param txPin The GPIO pin on the Raspberry Pi Pico to use for TX.
     *              Must be a valid pin for PIO-based UART. 
     *              For example, "16" for GP16.
     */
    DaisyUARTSend(int txPin)
    : pioSerial_(txPin, SerialPIO::NOPIN) // TX only, no RX pin
    {
        // Start the PIO-based serial port at 115200 baud
        pioSerial_.begin(115200);
    }

    /**
     * @brief SLIP-encodes the float vector and sends it as one packet.
     * @param params  Vector of floats to send to the Daisy.
     */
    void SendParams(const std::vector<float> &params)
    {
        // Encode each float (4 bytes) via SLIP
        for(const auto& f : params)
        {
            union {
                float   f;
                uint8_t b[4];
            } data;
            data.f = f;

            for(int i = 0; i < 4; i++)
            {
                slipSendByte(data.b[i]);
                //Serial.print(data.b[i]);
            }
        }

        // Finally, write the SLIP_END marker
        pioSerial_.write(SLIP_END);
        Serial.print(params[0], 5);
        Serial.print(", ");
        Serial.print(params[params.size()-1], 5);
        Serial.print(" ");
        Serial.print(params.size());
        Serial.println("");
    }

private:
    /**
     * @brief Helper to apply SLIP-escaping on a single byte.
     */
    void slipSendByte(uint8_t b)
    {
        if(b == SLIP_END)
        {
            pioSerial_.write(SLIP_ESC);
            pioSerial_.write(SLIP_ESC_END);
        }
        else if(b == SLIP_ESC)
        {
            pioSerial_.write(SLIP_ESC);
            pioSerial_.write(SLIP_ESC_ESC);
        }
        else
        {
            pioSerial_.write(b);
        }
    }

private:
    // PIO-based Serial
    SerialPIO pioSerial_;

    // SLIP special characters
    static constexpr uint8_t SLIP_END     = 0xC0;  ///< End of SLIP packet
    static constexpr uint8_t SLIP_ESC     = 0xDB;  ///< Escape character
    static constexpr uint8_t SLIP_ESC_END = 0xDC;  ///< Escaped END
    static constexpr uint8_t SLIP_ESC_ESC = 0xDD;  ///< Escaped ESC
};


const size_t kN_synthparams = 3;


#endif  // __DAISY_UART_SEND_HPP__

