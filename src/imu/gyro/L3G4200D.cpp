
#include <Arduino.h>
#include <Wire.h>
#include "L3G4200D.h"
#include "../imu.h"
#include "driver/gpio.h"

L3G4200D::L3G4200D()
{
	Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    /* Enable gyro (Normal mode - X,Y,Z axes) */
	writeTo(L3G4200D_CTRL_REG1, 0x0F);
    /* Full scale selection (2000 dps) */
    writeTo(L3G4200D_CTRL_REG4, 0x30);
}

void L3G4200D::writeTo(byte address, byte val)
{
	Wire.beginTransmission(L3G4200D_DEVICE);
	Wire.write(address);
	Wire.write(val);
	Wire.endTransmission();
}

void L3G4200D::readFrom(byte address, int num, byte _buff[])
{
	Wire.beginTransmission(L3G4200D_DEVICE);
	Wire.write(address);
	Wire.endTransmission();

	Wire.beginTransmission(L3G4200D_DEVICE);
	Wire.requestFrom(L3G4200D_DEVICE, num);

	int i = 0;
	while(Wire.available())
	{
		_buff[i] = Wire.read();
		i++;
	}
	/*if(i != num){
		status = L3G4200D_ERROR;
		error_code = L3G4200D_READ_ERROR;
	}*/
	Wire.endTransmission();
}

void L3G4200D::read()
{
    byte X_L = 0;
    byte X_H = 0;
    byte Y_L = 0;
    byte Y_H = 0;
    byte Z_L = 0;
    byte Z_H = 0;

	readFrom(L3G4200D_OUT_X_L, 1, _buff);
    X_L = _buff[0];
    readFrom(L3G4200D_OUT_X_H, 1, _buff);
    X_H = _buff[0];
    readFrom(L3G4200D_OUT_Y_L, 1, _buff);
    Y_L = _buff[0];
    readFrom(L3G4200D_OUT_Y_H, 1, _buff);
    Y_H = _buff[0];
    //readFrom(L3G4200D_OUT_Z_L, 1, _buff);
    //Z_L = _buff[0];
    //readFrom(L3G4200D_OUT_Z_H, 1, _buff);
    //Z_H = _buff[0];

	gyroVector[0] = ((int16_t)X_H << 8) | X_L;
	gyroVector[1] = ((int16_t)Y_H << 8) | Y_L;
	//gyroVector[2] = ((int16_t)Z_H << 8) | Z_L;

    gyroVector[0] = (float)getSignedNumber((int16_t)gyroVector[0]);
    gyroVector[1] = (float)getSignedNumber((int16_t)gyroVector[1]);
    //gyroVector[2] = (float)getSignedNumber((int16_t)gyroVector[2]);

	//gyroVector[0] *= -1;
	//gyroVector[1] *= -1;
	//gyroVector[2] *= -1;
}

int16_t L3G4200D::getSignedNumber(int16_t number)
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

void L3G4200D::scale()
{
	rateVector[0] = 0;
	rateVector[1] = 0;
	rateVector[2] = 0;

	rateVector[0] = gyroVector[0] * GYRO_SENSITIVITY;
	rateVector[1] = gyroVector[1] * GYRO_SENSITIVITY;
	//rateVector[2] = gyroVector[2] * GYRO_SENSITIVITY;
}

void L3G4200D::applyCalibration()
{

}

/*
 * getOrientationVector method:
 * store calibrated and scaled accelerometer data(x,y,z) in '&data' parameter
 */
void L3G4200D::getGyroVector(float (&data)[3])
{
	read();
	applyCalibration();
	scale();
	//Serial.println("X: " + String(gyroVector[0]) + ",Y: " + String(gyroVector[1]) + ",Z: " + String(gyroVector[2]));

	memcpy(data, gyroVector, sizeof(data));
}