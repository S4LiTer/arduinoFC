#include "quaternion.h"

int main(void) {
    quaternion q = quaternion(0, 0, 0);
    q.addEuler(1, 1, 1);

    float *a = q.GetEuler();

    cout << "\nX: " << a[0];
    cout << ", Y: " << a[1];
    cout << ", Z: " << a[2] << endl;
}