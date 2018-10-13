/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: IMU
 * Owner: gchinellato
 */

#ifndef IMU_H
#define IMU_H

#include "accel/ADXL345.h"
#include "gyro/L3G4200D.h"
#include "mag/HMC5883L.h"
#include "../pinmux/pinmux.h"
#include <math.h>

#define CF 				0.96

class GY80
{
public:
	GY80();
	float* getOrientation(int algorithm, float cf, float G_dt);
	void magCalibration();
	void accCalibration();
	void gyrCalibration();
private:
	ADXL345 accelerometer;
	L3G4200D gyro;
	HMC5883L magnetometer;
	float orientation[3]; //roll, pitch, yaw
	float accVector[3];
	float magVector[3];
	float compMagVector[3];
	float gyroVector[3];
	float magnetoHeading(float magnetometer[3], float accelerometer[3]);
	void complementaryFilter(float G_dt, float cf, float (&orientationDeg)[3]);
};

#endif