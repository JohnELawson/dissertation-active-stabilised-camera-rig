#include "servo.h"

//initializes servo controll object 
servo::servo(PinName tx, PinName rx, int id) 
    : conn(tx, rx) { 
   conn.baud(1000000);
   sid = id;
}

//initializes motors values 
void servo::initServo(int ccl, int cl, float crs){
    //set clock wise limit
    char data[2];
    short limit = (1023 * cl) / 300;
    data[0] = limit & 0xff; //bottom 8 bits
    data[1] = limit >> 8;   //top 8 bits    
    write(sid, 0x06, 2, data, false);   
    
    //set counter clockwise limit
    limit = (1023 * ccl) / 300;
    data[0] = limit & 0xff; //bottom 8 bits
    data[1] = limit >> 8;   //top 8 bits    
    write(sid, 0x08, 2, data, false);    
    
    //set cr speed
    int goal = 0xff * abs(crs);
    data[0] = goal & 0xff; //bottom 8 bits
    data[1] = goal >> 8;   //top 8 bits    
    write(sid, 0x20, 2, data, false); 
}

//sets the target position to the motor
void servo::setPos(int pos){
    char data[2];
    data[0] = pos & 0xFF;   //bottom 8 bits  
    data[1] = pos >> 8;     //top 8 bits  
    write(sid, GOAL_POS, 2, data, false);
}

//function to write packet to motor
int servo::write(int id, int addr, int length, char* data, bool reply){
    unsigned char msg[16];
    int sum = 0;
    
    //start bytes
    msg[0] = 0xFF;
    msg[1] = 0xFF;
    
    //id 
    msg[2] = id;
    sum += msg[2];
    
    //length
    msg[3] = 3 + length;
    sum += msg[3];
    
    //instruction
    msg[4] = 0x03;//write
    sum += msg[4];
    
    //start addr
    msg[5] = addr;
    sum += msg[5];
    
    //data                 
    for(int i=0; i<length; i++){
        msg[6+i] = data[i]; 
        sum += msg[6+i];
    }   
    
    //checksum
    msg[6+length] = 0xff - sum;
    
    //print message to serial
    for(int i=0; i<7+length; i++){
        conn.putc(msg[i]);       
    }
    
    wait(0.00002);

    return 0;
}
    