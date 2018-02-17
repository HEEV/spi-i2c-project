#include "WiiNunchuk.h"


WiiNunchuk::WiiNunchuk(I2C_HandleTypeDef *handle)
{
     i2c = handle;

     //Defaults for everyone.
     accelerometerX = 0x00;
     accelerometerY = 0x00;
     accelerometerZ = 0x00;
     analogStickX = 0x66;
     analogStickY = 0x00;
     cKeyDown = 1;
     zKeyDown = 1;
}
    

//This function requires that the I2C bus be set up and read to transmit.
void WiiNunchuk::nunchuckInit()
{
    //Start by sending the proper I2C commands to the Nunchuck to communicate.

    //We start by initing the nunchuk in encrypted mode.
    uint8_t buffer[2] = {0x40, 0x00};
    HAL_I2C_Master_Transmit(i2c, NUNCHUK_ADDRESS, buffer, 2, 100);
}

void WiiNunchuk::updateNunchuckData()
{
    uint8_t dataBuffer[6];
    uint8_t buffer[1] = {0x00};
    HAL_I2C_Master_Transmit(i2c, NUNCHUK_ADDRESS, buffer, 1, 100);
    HAL_Delay(1);
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

    decrypt(dataBuffer);

    accelerometerZ = (dataBuffer[4] << 2) | ((0xC0 & dataBuffer[5]) >> 6);
    accelerometerY = (dataBuffer[3] << 2) | ((0x30 & dataBuffer[5])  >> 4);
    accelerometerX = (dataBuffer[2] << 2) | ((0x0C & dataBuffer[5])  >> 2);
    analogStickY = dataBuffer[1];
    analogStickX = dataBuffer[0];
    cKeyDown = ((dataBuffer[5] & 0x02) >> 1);
    zKeyDown = (dataBuffer[5] & 0x01);
}


void WiiNunchuk::decrypt(uint8_t buffer[])
{
    for(uint8_t i = 0; i < 6; i++)
    {
        buffer[i] ^= 0x17; 
        buffer[i] += 0x17; 
    }
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
    return (cKeyDown==0) ? true : false;
}
        
bool WiiNunchuk::isZKeyDown()
{
    return (zKeyDown==0) ? true : false;
}
