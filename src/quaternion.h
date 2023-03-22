#ifndef QUATERNION
#define QUATERNION

#include <math.h>

using namespace std;

class quaternion {
    public:
        quaternion(float X, float Y, float Z);

        double R = 1;
        double i = 0;
        double j = 0;
        double k = 0;

        void addEuler(float X, float Y, float Z);
        float* GetEuler();
    private:

        float* euler2quaternion(float X, float Y, float Z);
        float vectorLength(float X, float Y, float Z);
};


#endif