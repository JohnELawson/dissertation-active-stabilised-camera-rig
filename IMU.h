#ifndef IMU_h
#define IMU_h

#include "mbed.h"

class IMU {
    private:
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset; 
        float ax_usable, ay_usable, az_usable, gx_usable, gy_usable, gz_usable; 
        float ax_filtered, ay_filtered, az_filtered, gx_filtered, gy_filtered, gz_filtered;
        float gx_unfiltered_last, gy_unfiltered_last, gz_unfiltered_last;
        float gx_unfiltered, gy_unfiltered, gz_unfiltered;
        float x_angle, y_angle, z_angle;
        float x_angle_last, y_angle_last, z_angle_last;
        float x_joy, y_joy; 
        float deltaTime;
        float rad2Deg;
        float alpha;
        Timer t;

    public:
        IMU();
        
        void initialize();
        void reset();
        
        int testConnection();
        void calibrateSensors();
        
        void getRawData();        
        void computeAngles(float *x_angle_return, float *y_angle_return, float *z_angle_return);
        
        void setMasterMode(bool en);
        void setI2CBypassMode(bool en);
        void setSlaveAddress(uint8_t regOffset, char address);
        void setSlaveRegister(uint8_t regOffset, char reg);
        void setSlaveControl(int regNo, char ctrl);
        
        void initializeMagnetometer();
        int testMagnetometerConnection();
        void magRawData();//uint16_t *mx, uint16_t *my, uint16_t *mz);
};

#endif