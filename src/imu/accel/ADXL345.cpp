/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Accelerometer
 * Owner: gchinellato
 */


#include <Arduino.h>
#include <Wire.h>
#include "ADXL345.h"
#include "../imu.h"
#include "driver/gpio.h"

/*
* object constructor:
* initialize I2C communication
* write 0x08 in register 0x2d (place the part in measurement mode)
*/
ADXL345::ADXL345()
{
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    /* Enable the accelerometer */
	writeTo(ADXL345_POWER_CTL, 0x08);
}

/*
* writeTo method:
* write 'val' in 'address' register (using I2C bus <Wire> lib)
*/
void ADXL345::writeTo(byte address, byte val)
{
	Wire.beginTransmission(ADXL345_DEVICE);
	Wire.write(address);
	Wire.write(val);
	Wire.endTransmission();
}

/*
* readFrom method:
* read 'num' bytes from starting 'address' register onwards and stores in '_buff[]'
* (using I2C bus <Wire> lib)
*/
void ADXL345::readFrom(byte address, int num, byte _buff[])
{
	Wire.beginTransmission(ADXL345_DEVICE);
	Wire.write(address);
	Wire.endTransmission();

	Wire.beginTransmission(ADXL345_DEVICE);
	Wire.requestFrom(ADXL345_DEVICE, num);

	int i = 0;
	while(Wire.available())
	{
		_buff[i] = Wire.read();
		i++;
	}
	/*if(i != num){
		status = ADXL345_ERROR;
		error_code = ADXL345_READ_ERROR;
	}*/
	Wire.endTransmission();
}

/*
* read method:
* store sensor data(x,y,z) in 'orientationVector'
*
* values in two's complement form
	 * X value: _buff[1] is MSB & _buff[0] is LSB
	 * Y value: _buff[3] is MSB & _buff[2] is LSB
	 * Z value: _buff[5] is MSB & _buff[4] is LSB
*/
void ADXL345::read()
{
	readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff);

	accVector[0] = (((int16_t)_buff[1]) << 8) | _buff[0];
	accVector[1] = (((int16_t)_buff[3]) << 8) | _buff[2];
	accVector[2] = (((int16_t)_buff[5]) << 8) | _buff[4];

    accVector[0] = (float)getSignedNumber((int16_t)accVector[0]);
    accVector[1] = (float)getSignedNumber((int16_t)accVector[1]);
    accVector[2] = (float)getSignedNumber((int16_t)accVector[2]);

    //Serial.println("0: " + String(accVector[0]) + ", 1: " + String(accVector[1])+ ", 2: " + String(accVector[2]));

	//accVector[0] *= -1;
    //accVector[1] *= -1;
    //accVector[2] *= -1;
}

int16_t ADXL345::getSignedNumber(int16_t number)
{
    if(number & (1 << 15))
    {
        number |= !0xFFFF;
    }
    else
    {
        number &= 0xFFFF;
    }
    return number;
}

void ADXL345::scale(int mode)
{
    /* Acceleration G-Force */
    accVector[0] *= ACC_SCALE_MULTIPLIER;
    accVector[1] *= ACC_SCALE_MULTIPLIER;
    accVector[2] *= ACC_SCALE_MULTIPLIER;

    if(mode)
    {
        /* Acceleration m/s2 */
        accVector[0] *= EARTH_GRAVITY_MS2;
        accVector[1] *= EARTH_GRAVITY_MS2;
        accVector[2] *= EARTH_GRAVITY_MS2;
    }
}

void ADXL345::applyCalibration()
{

}

void ADXL345::getAccAngles()
{
    /* 180 degrees - 2D */
    roll = atan2(accVector[1], accVector[2]);
    //pitch = atan2(-accVector[0], accVector[2]);

    /* 90 degrees - 3D */
    //roll = atan2(accVector[1], sqrt((accVector[0]*accVector[0]+accVector[2]*accVector[2])));
    pitch = atan2(-accVector[0], sqrt((accVector[1]*accVector[1]+accVector[2]*accVector[2])));

    //Serial.println("Roll: " + String(roll*RAD_TO_DEG) + ", Pitch: " + String(pitch*RAD_TO_DEG));
}

/*
* getOrientationVector method:
* store calibrated and scaled accelerometer data(x,y,z) in 'data' parameter
*/
void ADXL345::getAccVector(float (&data)[3])
{
	read();
	applyCalibration();
	scale(1);
    getAccAngles();

	memcpy(data, accVector, sizeof(data));
}