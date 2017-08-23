#ifndef servo_h
#define servo_h

#include "mbed.h"

class servo {
    private:
        Serial conn; 
        char sid;       

    public:
        servo(PinName tx, PinName rx, int id);
        
        void initServo(int ccl, int cl, float crs);
        void setPos(int pos);
        int write(int id, int addr, int length, char* data, bool reply);

};

#define GOAL_POS 0xFF

#endif