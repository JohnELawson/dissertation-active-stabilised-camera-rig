#ifndef I2C_CONN_h
#define I2C_CONN_h

#include "mbed.h"

//i2c pins
#define I2C_SDA                 p28
#define I2C_SCL                 p27

//imu
#define IMU_ADDRESS             0xD0//0x68 << 1
#define CONFIG                  0x1A
#define GYRO_CONFIG             0x1B
#define ACCELEROMETER_CONFIG    0x1C
#define RA_I2C_SLV0_ADDR        0x25
#define RA_I2C_SLV0_REG         0x26
#define RA_I2C_SLV0_CTRL        0x27
#define INTERRUPT_PIN_CONFIG    0x37
#define ACCEL_XOUT              0x3B
#define ACCEL_YOUT              0x3D
#define ACCEL_ZOUT              0x3F
#define GYRO_XOUT               0x43
#define GYRO_YOUT               0x45
#define GYRO_ZOUT               0x47
#define RA_USER_CTRL            0x6A
#define POWER_MANAGEMENT_1      0x6B
#define POWER_MANAGEMENT_2      0x6C
#define WHO_AM_I                0x75

//magnetometer
#define MAG_ADDRESS             0x3D
#define CONFIG_A                0x00
#define CONFIG_B                0x01
#define MODE_REG                0x02
#define RA_DATAX                0x03
#define RA_DATAZ                0x05
#define RA_DATAY                0x07
#define STATUS                  0x09
#define ID_REG_A                0x0A
//magnetometer average sample values
#define AVG1_SAMPLES            0x00
#define AVG2_SAMPLES            0x20
#define AVG4_SAMPLES            0x80
#define AVG8_SAMPLES            0xC0
//magnetometer output rate values
#define OUTPUT_RATE_0_75        0x00
#define OUTPUT_RATE_1_5         0x04
#define OUTPUT_RATE_3           0x08
#define OUTPUT_RATE_7_5         0x0C
#define OUTPUT_RATE_15          0x10
#define OUTPUT_RATE_30          0x14
#define OUTPUT_RATE_75          0x18

class I2C_CONN {
    private:
        I2C i2c;
        Serial debugSerial;
    public:
        I2C_CONN();
        I2C_CONN(PinName i2cSda, PinName i2cScl);        
        
        char i2c_read_byte(char slave_addr, char reg_addr);
        int i2c_write_byte(char slave_addr, char reg_addr, char data);
        
        int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
        int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);

        int testConnection();
        int testMagnetometerConnection();
};

#endif