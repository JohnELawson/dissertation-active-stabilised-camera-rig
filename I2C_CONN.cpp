#include "I2C_CONN.h"

//#define useDebugSerial

I2C_CONN::I2C_CONN(): i2c(I2C_SDA, I2C_SCL), debugSerial(USBTX, USBRX)
{

}

I2C_CONN::I2C_CONN(PinName i2cSda, PinName i2cScl): i2c(i2cSda, i2cScl), debugSerial(USBTX, USBRX)
{
    
}

//read one reg (1 byte)
char I2C_CONN::i2c_read_byte(char slave_addr, char reg_addr)
{
    char reg_data;
    
    i2c.write(slave_addr, &reg_addr, 1, true);
    i2c.read(slave_addr, &reg_data, 1);
    
    return reg_data;
}

//write one reg (1 byte)
int I2C_CONN::i2c_write_byte(char slave_addr, char reg_addr, char data)
{
    char dataArray[2] = {reg_addr, data}; 
    
    i2c.write(slave_addr, dataArray, 2); //write data array to reg

    return 1;
}


//read multiple registers
int I2C_CONN::i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    char signed_reg_addr = (signed) reg_addr;
    int int_length = (int) length;
    
    char *read_data = (char*)malloc(int_length); //allocates meomy space for length of data to read
    
    i2c.write(slave_addr, &signed_reg_addr, 1, true); //select starting point
    i2c.read(slave_addr, read_data, int_length);    //read data
    
    for(int i=0; i<length; i++) { //loop through read data and copy to return data address
        data[i] = read_data[i];
    }
    free (read_data); //free mem allocation
    
    return 1;
}


//write multiple registers
int I2C_CONN::i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data){
    for(int i=0; i<=length; i++){
        i2c_write_byte(slave_addr, reg_addr + i, data[i]);
    }  
    return 1; 
//    i2c.start();
//    i2c.write(slave_addr<<1);
//    i2c.write((signed)reg_addr);
//    for(int i = 0; i < length; i++) {
//        i2c.write(data[i]);
//    }
//    i2c.stop();
//    
//    return 1;
}


int I2C_CONN::testConnection(){
    //check this, need to loop all addresses at begining otherwise weird errors occur
    unsigned char data;
    
    for(int addr = 0; addr < 118; addr++){
        data = i2c_read_byte(0xD0, addr);//IMU_ADDRESS<<1, addr);
        //printf("reg addr: %d - 0x%x   |   data = %d - 0x%x\n", addr, addr, data, data); //print every reg data
    } 
    
    //read WHO_AM_I 
    i2c_read(IMU_ADDRESS, WHO_AM_I, 1, &data);
    return (data == 0x68 ? 1 : 0); //reads the WHO_AM_I reg on the IMU 
}

int I2C_CONN::testMagnetometerConnection(){
    unsigned char data;
    
    for(int addr = 0; addr < 12; addr++){
        data = i2c_read_byte(MAG_ADDRESS, addr);//0x18, addr);
        //printf("reg addr: %d - 0x%x   |   data = %d - 0x%x\n", addr, addr, data, data); //print every reg data
    } 
    
    //read identification register A 
    i2c_read(MAG_ADDRESS, ID_REG_A, 1, &data); //0x18, 0x00
    return (data == 0x48 ? 1 : 0); //reads the WHO_AM_I reg on the IMU 
}