#include "MPU6050.h"
#include "oppie.h"

MPU6050 mpu;

// Motor A 72 (Left)90
// Motor B 61 (Right)70

void setup()
{
 Serial.begin(9600);

 motor_init();
 //calibrate_motors();

 set_motor_A(100);
 set_motor_B(100);
// motor speed between -255 and 255
 

 mpu.initialize();
 init_filter();
 mpu.setXAccelOffset(-7);
 mpu.setYAccelOffset(-2357);
 mpu.setZAccelOffset(395);

 mpu.setXGyroOffset(142);
 mpu.setYGyroOffset(-18);
 mpu.setZGyroOffset(-70);

 
}

void loop()
{
/*
 Serial.print(mpu.getAccelerationX());
 Serial.print(",");
 Serial.print(mpu.getAccelerationY());
 Serial.print(",");
 Serial.print(mpu.getAccelerationZ());
 Serial.println();
*/

/*
Serial.print(mpu.getRotationX());
Serial.print(",");
Serial.print(mpu.getRotationY());
Serial.print(",");
Serial.print(mpu.getRotationZ());
Serial.println();
*/

updateAcceleration(mpu.getAccelerationZ(), mpu.getAccelerationY());
updateRotation(mpu.getRotationX());

Serial.print(getAccelerationAngle());
Serial.print(",");
Serial.print(getGyroscopeAngle());
Serial.print(",");
Serial.print(getFilteredAngle());
Serial.println();

// setpoint 1.10 to 1.25
 
 
 /*
 updateAcceleration(mpu.getAccelerationZ(), mpu.getAccelerationY());
 updateRotation(mpu.getRotationX());

 
 */
}
