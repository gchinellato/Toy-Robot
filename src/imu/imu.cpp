/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: IMU
 * Owner: gchinellato
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "imu.h"

GY80::GY80()
{
    /* time for sensors to start */
    delay(50); 
}

void GY80::magCalibration()
{

}

float* GY80::getOrientation(int algorithm, float cf, float G_dt)
{
    /* roll, pitch, yaw */    
    complementaryFilter(G_dt, cf, orientation);
	return orientation;
}

void GY80::complementaryFilter(float G_dt, float cf, float (&orientationDeg)[3])
{
    accelerometer.getAccVector(accVector);
    gyro.getGyroVector(gyroVector);

    //magnetometer.getMagVector(magVector);
    //magnetometer.tiltCompensation(accelerometer.roll, accelerometer.pitch, compMagVector);
    //magnetometer.setDeclination(-21.0, 7.0);
    //magnetometer.getHeading(compMagVector[0], compMagVector[1], true);

    /* CF = tau / (tau+LP)
      tau = CF*LP/(1-CF)
	  i.e: 0.98*0.01sec/(1-0.98) = 0.49tau
      (if the loop period is shorter than this value, gyro take precedence, otherwise, acceleromter is given more weighting than gyro)
      orientation in degrees (pitch, roll, yaw from rotation matrix) */
    orientationDeg[0] = cf*(orientationDeg[0] + gyro.rateVector[0]*G_dt) + (1-cf)*accelerometer.roll*RAD_TO_DEG;
    orientationDeg[1] = cf*(orientationDeg[1] + gyro.rateVector[1]*G_dt) + (1-cf)*accelerometer.pitch*RAD_TO_DEG;
    //orientationDeg[2] = cf*(orientationDeg[2] + gyro.rateVector[2]*G_dt) + (1-cf)*magnetometer.heading*RAD_TO_DEG;
}