#ifndef KRUPS_SENSOR_H
#define KRUPS_SENSOR_H

#include <SparkFunMPU9250-DMP.h>

#include "Control.h"
#include "Config.h"

// Set correction values appropriately to zero sensors when in a
// known situation e.g. board laying on a level, unmoving surface
#define gyroXcorr     (0)
#define gyroYcorr     (0)
#define gyroZcorr     (0)
#define accelXcorr    (0)
#define accelYcorr    (0)
#define accelZcorr    (0)
#define hiXcorr       (0)
#define hiYcorr       (0)
#define hiZcorr       (0)
#define magXcorr      (0)
#define magYcorr      (0)
#define magZcorr      (0)

//High accel pins
#define ADXL377x    (17)
#define ADXL377y    (16)
#define ADXL377z    (15)

MPU9250_DMP senser;

//Initializes Accelerator, Gyrometer sensors
void Init_Accel_Gyro (void)
{
  senser.begin();
  senser.setAccelFSR(16);
  senser.setGyroFSR(2000);

}
