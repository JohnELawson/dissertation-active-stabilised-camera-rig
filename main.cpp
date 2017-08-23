#include "mbed.h"
#include "IMU.h"
#include "pid.h"
#include "AX12.h"
//#include "servo.h"

//USB serial and mBed light
Serial pc(USBTX, USBRX);  
DigitalOut led1(LED1); 

//IMU object
IMU imu;                            

//SERVO connection objects
AX12 motor_y (p13, p14, 3);         
AX12 motor_x (p13, p14, 2);           
AX12 motor_z (p13, p14, 1);       
//servo servo_y(p13, p14, 1);
 
//PID calculator objects
pid pid_y(0.2, 0.0002, 0.00001);   
pid pid_x(0.205, 0.0002, 0.00001);    
pid pid_z(0.2, 0.0002, 0.00001);    

//init variables
float angle_x, angle_y, angle_z;    //IMU angles
int out_y, out_x, out_z;            //motor values

int main() 
{   
    //setup pc serial
    pc.baud(921600);
    wait(0.5);
    printf("\n\nStart\n\n");
    
    //test IMU connection
    if(imu.testConnection())
    {
        printf("Connected To IMU\n");
        
        printf("Initializing IMU...\n");  
        imu.initialize();     
        
        printf("Calibrating Sensors\n\r");
        imu.calibrateSensors();
        
        //switch from imu to magnetometer
        imu.setMasterMode(false);
        imu.setI2CBypassMode(true);
        
        //test magnetometer connection
        if(imu.testMagnetometerConnection()){
            printf("Connected To Magnetometer\n");
            
            printf("Initializing Magnetometer\n");  
            imu.initializeMagnetometer();
            
            //linking magnetometer registers to IMU registers didnt work
//            //go back to imu
//            imu.setI2CBypassMode(false);
//            
//            //link magno reg X to imu register
//            imu.setSlaveAddress(0, MAG_ADDRESS | 0x80);//0x1E);        //set reg to read from magno
//            imu.setSlaveRegister(0, RA_DATAX);          //reg addr of target magno x 
//            imu.setSlaveControl(0, 0x80 | 0x02);        //enable slave address | data length 2 bytes
//            
//            //link magno reg Y to imu register
//            imu.setSlaveAddress(1, 0x1E | 0x80);
//            imu.setSlaveRegister(1, RA_DATAY);
//            imu.setSlaveControl(1, 0x80 | 0x02);
//            
//            //link magno reg Z to imu register
//            imu.setSlaveAddress(2, 0x80 | 0x1E);
//            imu.setSlaveRegister(2, RA_DATAZ);
//            imu.setSlaveControl(2, 0x80 | 0x02);
//            
//            imu.setMasterMode(true);
            
            printf("Preparing Motors\n");  
             
            //init motors to middle of each axis
            out_y = 512;
            out_x = 512;
            out_z = 512;
            motor_y.SetGoal(out_y);
            motor_x.SetGoal(out_x);
            motor_z.SetGoal(out_z);
            wait(0.3);
            
            printf("Starting Main Loop...\n\n");  
            
            //set id of motor
//            while(0){
//                float i = motor_y.GetVolts();
//                motor_x.SetID(1, 3);
//                printf("set %f\n", i);
//                wait(1);
//            }

//            //test magnetometer 
//            imu.setMasterMode(false);
//            imu.setI2CBypassMode(true);
//            while(1){
//                int16_t raw[3];
//                mag.getXYZ(raw);
//                double heading = atan2((double)raw[2], (double)raw[0]);
//                printf("%.3f, %.3f, %d, %d, %d  \n", (heading * (180/3.14159)), (raw[0] * (180/3.14159)), raw[0], raw[1], raw[2]);
//            }

//            uint16_t mx, my, mz;
//            while(1){
//                imu.magRawData(&mx, &my, &mz);    
//                printf("%d, %d, %d \n",mx, my, mz);
//            }

            //Timer tt;
            //tt.start();
            
            //### main program loop ### ~ 5ms + wait at end of loop
            while(1){
                //get IMU raw values and calculate angles
                imu.computeAngles(&angle_x, &angle_y, &angle_z);
                
                //compute pid functions for each motor 
                out_y += pid_y.compute(-angle_y);
                out_x += pid_x.compute(-angle_x);
                out_z += pid_z.compute(angle_z);
                
                //servo bounds - stops rig hitting next axis     
                if(out_y > 800) out_y = 800;
                else if(out_y < 200) out_y = 200;
                if(out_x > 800) out_x = 800;
                else if(out_x < 200) out_x = 200;
                if(out_z > 800) out_z = 800;
                else if(out_z < 200) out_z = 200;
                
                //set motor position
                motor_y.SetGoal(out_y);
                motor_x.SetGoal(out_x);
                //motor_z.SetGoal(out_z);
                
                //update usb serial of current values
                //printf("%d  \r", tt.read_us());
                //tt.reset();
                printf("   ay:%.3f, ax:%.3f, az:%.3f  |  y:%d, x:%d, z:%d   \r", angle_y, angle_x, angle_z,    out_y, out_x, out_z);
                 
                //wait_ms(10);       
            }
        }
        else {
            printf("Failed To Connect To Magnetometer\n"); 
            imu.reset();   
            
            while(1) {//flash light on error for feedback
                led1 = 1; wait(0.1);
                led1 = 0; wait(0.1);
            }
        }  
    }
    else 
    {
        printf("Failed To Connect To IMU\n");
        imu.reset();
        
        while(1) {//flash light on error for feedback
            led1 = 1; wait(0.5);
            led1 = 0; wait(0.5);
        }
    }   

}