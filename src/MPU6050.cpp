#include "MPU6050.h"

MPU6050::MPU6050(int MPUaddress) {
    address = MPUaddress;
}


void MPU6050::begin() {
    Wire.begin();
    Wire.beginTransmission(address);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    /*
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);
    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
    delay(20);
    */
    status = 's';
}


void MPU6050::read() {
    if(!LastMeasurementTime) {
        LastMeasurementTime = micros();
        return;
    }

    Wire.beginTransmission(address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true);

    AccX = (Wire.read() << 8 | Wire.read() ) / AccDivident;
    AccY = (Wire.read() << 8 | Wire.read() ) / AccDivident;
    AccZ = (Wire.read() << 8 | Wire.read() ) / AccDivident;


    TimeElapsed = (double) (micros() - LastMeasurementTime) / 1000000;
    LastMeasurementTime = micros();

    Wire.beginTransmission(address);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true);


    GyroX = ((Wire.read() << 8 | Wire.read() ) / GyroDivident) * deg2rad - GyroOfferX;
    GyroY = ((Wire.read() << 8 | Wire.read() ) / GyroDivident) * deg2rad - GyroOfferY;
    GyroZ = ((Wire.read() << 8 | Wire.read() ) / GyroDivident) * deg2rad - GyroOfferZ;
    
    gyroAngle.addEuler(GyroX*TimeElapsed, GyroY*TimeElapsed, GyroZ*TimeElapsed);
}



void MPU6050::FindGyroOffset(int samples, int delay_between_samples) {

    float gyroSumX = 0;
    float gyroSumY = 0;
    float gyroSumZ = 0;
    
    for(int s = 0; s < samples; s++) {
        Wire.beginTransmission(address);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(address, 6, true);

        gyroSumX += ((Wire.read() << 8 | Wire.read() ) / GyroDivident) * deg2rad;
        gyroSumY += ((Wire.read() << 8 | Wire.read() ) / GyroDivident) * deg2rad;
        gyroSumZ += ((Wire.read() << 8 | Wire.read() ) / GyroDivident) * deg2rad;


        delay(delay_between_samples);
    }
    GyroOfferX = gyroSumX / samples;
    GyroOfferY = gyroSumY / samples;
    GyroOfferZ = gyroSumZ / samples;
}