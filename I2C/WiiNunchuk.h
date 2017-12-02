#include <stm32f0xx_hal.h>
#include <stdbool.h>

#ifndef _NUNCHUK_DRV_H
#define _NUNCHUK_DRV_H

class WiiNunchuk
{
public:
        WiiNunchuk(I2C_HandleTypeDef *handle);
        void nunchuckInit();
        void updateNunchuckData();
        uint16_t GetAccelerometerX();
        uint16_t GetAccelerometerY();
        uint16_t GetAccelerometerZ();
        uint8_t GetAnalogStickX();
        uint8_t GetAnalogStickY();
        bool isCKeyDown();
        bool isZKeyDown();

private:

        void decrypt(uint8_t buffer[]);

        I2C_HandleTypeDef *i2c;
        const uint16_t NUNCHUK_ADDRESS = 0xA4;
        const uint16_t NUNCHUK_READ_ADDR = 0xA5;
        volatile uint16_t accelerometerX;
        volatile uint16_t accelerometerY;
        volatile uint16_t accelerometerZ;
        volatile uint8_t  analogStickX;
        volatile uint8_t  analogStickY;
        volatile bool cKeyDown;
        volatile bool zKeyDown;
};

#endif //_NUNCHUCK_DRV_H
