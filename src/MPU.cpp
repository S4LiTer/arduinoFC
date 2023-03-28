#include "MPU.h"

MPU::MPU(int MPUaddress) {
    address = MPUaddress;
}


void MPU::begin() {
    Wire.begin();
    Wire.beginTransmission(address);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();     
}


void MPU::read() {
    if(!LastMeasurementTime || LastMeasurementTime > micros()) {
        LastMeasurementTime = micros();
        return;
    }

    Wire.beginTransmission(address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true);

    AccY = ((Wire.read() << 8 | Wire.read() ) / AccDivisor) - acc_offset_y;
    AccX = ((Wire.read() << 8 | Wire.read() ) / AccDivisor) - acc_offset_x;
    AccZ = ((Wire.read() << 8 | Wire.read() ) / AccDivisor) - acc_offset_z;
    

    float acc_total_vector = sqrt( (AccX*AccX)+(AccY*AccY)+(AccZ*AccZ) );
    acc_angle_Y = asin(AccY/acc_total_vector);
    acc_angle_X = asin(AccZ/acc_total_vector);
    acc_angle_Z = -asin(AccX/acc_total_vector);
    
    if(status != 's') {
        gyroAngle.RewriteQuaternion(acc_angle_X, 0, acc_angle_Z);
    }


    TimeElapsed = (double) (micros() - LastMeasurementTime) / 1000000;
    LastMeasurementTime = micros();

    Wire.beginTransmission(address);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true);


    GyroY = ((Wire.read() << 8 | Wire.read() ) / GyroDivisor) * deg2rad - GyroOffsetX;
    GyroX = ((Wire.read() << 8 | Wire.read() ) / GyroDivisor) * deg2rad - GyroOffsetY;
    GyroZ = ((Wire.read() << 8 | Wire.read() ) / GyroDivisor) * deg2rad - GyroOffsetZ;
    
    gyroAngle.addEuler(GyroX*TimeElapsed, GyroY*TimeElapsed, GyroZ*TimeElapsed);

    status = 's';
    ComplementaryFilter();
}




void MPU::ComplementaryFilter() {
    float acc_multiplier = 1 - complementary_filter_gyro_multiplier;

    float* gyro_angles = gyroAngle.GetEuler();
    total_angle[0] = complementary_filter_gyro_multiplier*gyro_angles[0] + acc_multiplier*acc_angle_X;
    total_angle[2] = complementary_filter_gyro_multiplier*gyro_angles[2] + acc_multiplier*acc_angle_Z;

    total_angle[1] = gyro_angles[1]; //needs to be merged with magnetometer

    delete[] gyro_angles;

    gyroAngle.RewriteQuaternion(total_angle[0], 0, total_angle[2]);

    total_angle[0] = total_angle[0] * (1/deg2rad);
    total_angle[1] = total_angle[1] * (1/deg2rad);
    total_angle[2] = total_angle[2] * (1/deg2rad);
}





void MPU::FindGyroOffset(int samples, int delay_between_samples) {

    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    
    for(int s = 0; s < samples; s++) {
        Wire.beginTransmission(address);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(address, 6, true);

        sumX += ((Wire.read() << 8 | Wire.read() ) / GyroDivisor) * deg2rad;
        sumY += ((Wire.read() << 8 | Wire.read() ) / GyroDivisor) * deg2rad;
        sumZ += ((Wire.read() << 8 | Wire.read() ) / GyroDivisor) * deg2rad;


        delay(delay_between_samples);
    }
    GyroOffsetX = sumX / samples;
    GyroOffsetY = sumY / samples;
    GyroOffsetZ = sumZ / samples;
}

void MPU::FindAccOffset(int samples, int delay_between_samples) {
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    
    for(int s = 0; s < samples; s++) {
        Wire.beginTransmission(address);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(address, 6, true);

        sumX += ((Wire.read() << 8 | Wire.read() ) / AccDivisor);
        sumY += ((Wire.read() << 8 | Wire.read() ) / AccDivisor);
        sumZ += ((Wire.read() << 8 | Wire.read() ) / AccDivisor);


        delay(delay_between_samples);
    }
    acc_offset_x = (sumX / samples) - 1;
    acc_offset_y = (sumY / samples);
    acc_offset_z = (sumZ / samples);
}
