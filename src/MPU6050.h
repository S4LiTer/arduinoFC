#ifndef MPU6050_SETUP
#define MPU6050_SETUP

#include <Wire.h>
#include "quaternion.h"

class MPU6050 {
    public:
        MPU6050(int MPUaddress);
        void read();
        void begin();

        void FindGyroOffset(int samples, int delay_between_samples);

        quaternion gyroAngle = quaternion(0, 0, 0);
        quaternion accAngle = quaternion(0, 0, 0);
        quaternion totalAngle = quaternion(0, 0, 0);

        char status = 'n';

        float AccX, AccY, AccZ;
        float GyroX, GyroY, GyroZ;

    private:
        int address;
        float AccDivisor = 16384;
        float GyroDivisor = 131;
        float deg2rad = M_PI/180;


        float GyroOffsetX = 0; float GyroOffsetY = 0; float GyroOffsetZ = 0;

        unsigned long LastMeasurementTime = 0;
        double TimeElapsed;
};


//void delay(int _delay);
#endif