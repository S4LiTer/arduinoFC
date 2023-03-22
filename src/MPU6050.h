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
        float AccDivident = 16384;
        float GyroDivident = 131;
        float deg2rad = M_PI/180;

        
        float GyroOfferX = 0; float GyroOfferY = 0; float GyroOfferZ = 0;

        unsigned long LastMeasurementTime = 0;
        double TimeElapsed;
};


//void delay(int _delay);
#endif