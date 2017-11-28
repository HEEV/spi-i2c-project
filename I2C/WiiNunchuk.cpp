#include "WiiNunchuk.h"


WiiNunchuk::WiiNunchuk(I2C_HandleTypeDef *handle)
{
     i2c = handle;

     //Defaults for everyone.
     accelerometerX = 0x00;
     accelerometerY = 0x00;
     accelerometerZ = 0x00;
     analogStickX = 0x00;
     analogStickY = 0x00;
     cKeyDown = false;
     zKeyDown = false;
}
    

//This function requires that the I2C bus be set up and read to transmit.
void WiiNunchuk::nunchuckInit()
{
    //Start by sending the proper I2C commands to the Nunchuck to communicate.

    //We start by disabling encryption on the data.
//    uint8_t buffer[2] = {0xF0, 0x55};
    uint8_t buffer[2] = {0x40, 0x00};
    HAL_I2C_Master_Transmit(i2c, NUNCHUK_ADDRESS, buffer, 2, 100);

    //buffer[0] = 0xFB;
    //buffer[1] = 0x00;
    //Now actually init the controller.
    //HAL_I2C_Master_Transmit(i2c, NUNCHUK_ADDRESS, buffer, 2, 100);
}

void WiiNunchuk::updateNunchuckData()
{
    uint8_t dataBuffer[6];
    uint8_t buffer[1] = {0x00};
    HAL_I2C_Master_Transmit(i2c, NUNCHUK_ADDRESS, buffer, 1, 100);
    HAL_I2C_Master_Receive(i2c, NUNCHUK_ADDRESS, dataBuffer, 6, 100);

    //At this point we should have the data now to populate the fields with it.
    /*What you get in return is described in this overview:
    Bit Byte 7 6 5 4 3 2 1 0
    0 Joystick X-Axis [7:0]
    1 Joystick Y-Axis [7:0]
    2 Accelerometer X-Axis [9:2]
    3 Accelerometer Y-Axis [9:2]
    4 Accelerometer Z-Axis [9:2]
    5 AZ<1:0> AY<1:0> AX<1:0> /BC /BZ
    
    The BC and BZ are the inverse of the actual value.
    */

    accelerometerZ = (buffer[4] << 2) | ((0xC0 & buffer[6]) >> 6);
    accelerometerY = (buffer[3] << 2) | ((0x30 & buffer[6])  >> 4);
    accelerometerX = (buffer[2] << 2) | ((0x0C & buffer[6])  >> 2);
    analogStickY = buffer[1];
    analogStickX = buffer[0];
    cKeyDown = !((0x02 & buffer[6]) >> 1);
    zKeyDown = !(0x01 & buffer[6]);
}


uint16_t WiiNunchuk::GetAccelerometerX()
{
    return accelerometerX;
}
        
uint16_t WiiNunchuk::GetAccelerometerY()
{
    return accelerometerY;
}
        
uint16_t WiiNunchuk::GetAccelerometerZ()
{
    return accelerometerZ;
}
        
uint8_t WiiNunchuk::GetAnalogStickX()
{
    return analogStickX;
}
        
uint8_t WiiNunchuk::GetAnalogStickY()
{
    return analogStickY;
}
        
bool WiiNunchuk::isCKeyDown()
{
    return cKeyDown;
}
        
bool WiiNunchuk::isZKeyDown()
{
    return zKeyDown;
}
