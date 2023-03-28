#ifndef MPU_SETUP
#define MPU_SETUP

#include <Wire.h>
#include <quaternion.h>

class MPU {
    public:
        MPU(int MPUaddress);
        void read();
        void begin();

        void FindGyroOffset(int samples, int delay_between_samples);
        void FindAccOffset(int samples, int delay_between_samples);

        float total_angle[3] = {0, 0, 0};

        char status = 'n';




        float complementary_filter_gyro_multiplier = 0.9;
        
        float acc_offset_x = 0; float acc_offset_y = 0; float acc_offset_z = 0;
    private:
        quaternion gyroAngle = quaternion(0, 0, 0);
        float GyroX, GyroY, GyroZ;


        float AccX, AccY, AccZ;
        float acc_angle_X; float acc_angle_Y; float acc_angle_Z;


        void ComplementaryFilter();

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