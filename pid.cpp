#include "pid.h"

//initializes pid object
pid::pid(float p, float i, float d)
{
    Kp = p; 
    Ki = i;
    Kd = d;
    preError = 0;
    timer.start();
}

//function calculates pid value
short pid::compute(float error)
{
    //get time of loop - convert to seconds
    t = (float) timer.read_us()/1000000; 
    timer.reset();
    
    //compute
    error = error / 0.29;
    Pt = Kp * error;
    sum += error;
    It = Ki * sum * t;
    Dt = Kd * (error - preError) /t;
    
    output = (short) Pt  + Dt + It;
    
    //printf("%d, t:%.4f, err:%.3f, pT:%.3f, dT:%.3f, iT:%.3f    \r", output, t, error, Pt, Dt, It);
    
    preError = error;
    
    return output;
}