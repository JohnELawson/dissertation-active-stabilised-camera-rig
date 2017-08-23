#ifndef pid_h
#define pid_h

#include "mbed.h"

class pid {
    private:
        float Kp, Kd, Ki, Pt, It, Dt;
        float t, preError, sum;
        Timer timer;
        short output;

    public:
        pid(float p, float i, float d);
        
        short compute(float error);

};

#endif