#include "IMU.h"
#include "I2C_CONN.h"

//joystick analog lines
AnalogIn x_joy_axis(A4);        
AnalogIn y_joy_axis(A5);

//raw variables for gyro and accel readings
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, mx_raw, my_raw, mz_raw;

//usable values - corrected with offsets and to degrees/second
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset, mag_offset = 0;
float ax_usable, ay_usable, az_usable, gx_usable, gy_usable, gz_usable;
float mx_usable, my_usable, mz_usable, mag_X, mag_Y, mag_Z; 

//filtered values
float ax_filtered, ay_filtered, az_filtered, gx_filtered, gy_filtered, gz_filtered;
double mag_filtered;
float gx_unfiltered_last, gy_unfiltered_last, gz_unfiltered_last;
float gx_unfiltered, gy_unfiltered, gz_unfiltered;

//real computed x, y, z angles for current IMU position 
float x_angle, y_angle, z_angle;
float x_angle_last, y_angle_last, z_angle_last, mag_filtered_last;

 //variables for joystick offset
float x_joy, y_joy;            

//variables for filter
float deltaTime;
float rad2Deg;
float Pi2;
float alpha;

//temp counter
int lll = 0;

Timer t;

//initialize imu object
IMU::IMU()
{
    deltaTime = 0.01;
    rad2Deg = 180/3.14159;
    Pi2 = 3.1415982*2;
    alpha = 0.96;
    x_joy = 0;
    y_joy = 0;
}

//create i2c object
I2C_CONN conn(p28, p27);

//initialize imu function
void IMU::initialize(){
    conn.i2c_write_byte(IMU_ADDRESS, POWER_MANAGEMENT_1, 0x49);   //set up clock source
    conn.i2c_write_byte(IMU_ADDRESS, GYRO_CONFIG, 0x00);          //set up gyro sensitivity     //250/s
    conn.i2c_write_byte(IMU_ADDRESS, ACCELEROMETER_CONFIG, 0x01); //set up accel sensitivity    //+2g
    conn.i2c_write_byte(IMU_ADDRESS, POWER_MANAGEMENT_1, 0x09);   //disable sleep mode          //sleep mode    
}

//reset imu function
void IMU::reset(){
    conn.i2c_write_byte(IMU_ADDRESS, POWER_MANAGEMENT_1, 0xC9);
}

//calls i2c test connection function
int IMU::testConnection(){
    return conn.testConnection(); 
}

//get values from IMU
void IMU::getRawData(){
    unsigned char buffer[14];
    conn.i2c_read(IMU_ADDRESS, 0x3B, 14, buffer); 
    
    //split and merge IMU values - two 8 bit values into one 16bit
    ax_raw = (( (int16_t)buffer[0]) << 8) | buffer[1];
    ay_raw = (( (int16_t)buffer[2]) << 8) | buffer[3];
    az_raw = (( (int16_t)buffer[4]) << 8) | buffer[5];
    //ignore temperature buffer[6] [7]                          
    gx_raw = (( (int16_t)buffer[8]) << 8) | buffer[9]; 
    gy_raw = (( (int16_t)buffer[10]) << 8) | buffer[11];
    gz_raw = (( (int16_t)buffer[12]) << 8) | buffer[13];
    
    //magnetometer
//    unsigned char mag_buffer[6];
//    conn.i2c_read(0x18, 0x03, 6, mag_buffer);  //0x0C<<1 = 0x18
//    mx_raw = (( (int16_t)mag_buffer[0]) << 8) | mag_buffer[1];
//    my_raw = (( (int16_t)mag_buffer[2]) << 8) | mag_buffer[3];
//    mz_raw = (( (int16_t)mag_buffer[4]) << 8) | mag_buffer[5];
    
    //printf("%d %d %d \t\t |  %d %d %d \t\t | %d %d %d \n", mx_raw, my_raw, mz_raw,      gx_raw, gy_raw, gz_raw,     ax_raw, ay_raw, az_raw);
}    

//calabrates offsets for IMU sensors - sets readings to zero
void IMU::calibrateSensors(){
    int16_t NoOfReadings = 100;
    
    //read a few values and add together
    for(int i=0; i<NoOfReadings; i++){
        getRawData();
        
        ax_offset += ax_raw;
        ay_offset += ay_raw;
        az_offset += az_raw;
        gx_offset += gx_raw;  
        gy_offset += gy_raw;  
        gz_offset += gz_raw;  
    }
    
    //devide by number of to get average 
    ax_offset /= NoOfReadings;
    ay_offset /= NoOfReadings;
    az_offset /= NoOfReadings;
    gx_offset /= NoOfReadings;
    gy_offset /= NoOfReadings;
    gz_offset /= NoOfReadings;
}

//complementary filter
void IMU::computeAngles(float *x_angle_return, float *y_angle_return, float *z_angle_return){
    
    //read joystick values, do math to cut out uncertainty to stop drift
    x_joy += (float) ( (int)(x_joy_axis.read() * 100 - 50) ) / 10000;
    y_joy += (float) ( (int)(y_joy_axis.read() * 100 - 50) ) / 10000;
    
    //switch access to imu
    setI2CBypassMode(false); 
    setMasterMode(true);
    //get raw IMU data
    getRawData();
    
    //switch access to magnetometer
    setMasterMode(false); 
    setI2CBypassMode(true);
    //get magnetometer data
    magRawData();//&mx_usable, &my_usable, &mz_usable);
    
    //convert raw with offsets and calculate degrees per second for gyro
    ax_usable = (float) ax_raw; //- ax_offset;//) / 8192;
    ay_usable = (float) ay_raw; //- ay_offset;//) / 8192;
    az_usable = (float) az_raw; //- az_offset;//) / 8192;  
    gx_usable = (float) (gx_raw - gx_offset) / 131; 
    gy_usable = (float) (gy_raw - gy_offset) / 131;
    gz_usable = (float) (gz_raw - gz_offset) / 131;
            
    //filtered accelerometer angles in degrees
    ay_filtered = atan(-1 * ax_usable / sqrt(pow(ay_usable, 2) + pow(az_usable, 2))) * rad2Deg;
    ax_filtered = atan(ay_usable / sqrt(pow(ax_usable, 2) + pow(az_usable, 2))) * rad2Deg;
    az_filtered = atan(sqrt(pow(ax_usable, 2) + pow(ay_usable, 2)) / az_usable) * rad2Deg; 

                  
    //get time of loop, devide by 1000 for us to ms, devide by 1000 ms to sec
    t.stop();
    deltaTime = (float) t.read_us()/1000000; 
    t.reset();
    t.start();//reset and restart timer for next loops calculations
    
    //filtered gyroscope angles in degrees for current time sample period
    gx_filtered = gx_usable * deltaTime + x_angle_last;
    gy_filtered = gy_usable * deltaTime + y_angle_last;
    gz_filtered = gz_usable * deltaTime + z_angle_last;
    
    //compute complementary filter
    alpha = 0.96;
    x_angle = (alpha * gx_filtered + (1.0 - alpha) * ax_filtered) ;   
    y_angle = (alpha * gy_filtered + (1.0 - alpha) * ay_filtered) + (10 * x_joy);  
    //cannot compute
    z_angle = gz_filtered + (10 * y_joy); //(alpha * gz_filtered + (1.0 - alpha) * az_filtered) - (10 * y_joy); 
    
    //magnetometer calculations 
    //pitch  = y_angle, roll = x_angle
    float tilt_mag_x = (cos(y_angle)*mx_raw) + ((sin(y_angle)*cos(x_angle))*mx_raw) + ((sin(y_angle)*cos(x_angle)*mx_raw));
    float tilt_mag_y = (cos(x_angle)*my_raw) - (sin(x_angle)*my_raw);
    float tilt_mag_z = (-sin(y_angle)*mz_raw) + ((cos(y_angle)*sin(x_angle))*mz_raw) + ((cos(y_angle)*cos(x_angle))*mz_raw);
    float mag_unfiltered = atan2((double)my_raw, (double)mx_raw) * rad2Deg;
    float mag_filtered = (atan2(tilt_mag_y, tilt_mag_x) * rad2Deg) ;
    
    if(mag_filtered < 0.0) // fix sign
        mag_filtered += Pi2;
    if(mag_filtered > Pi2) // fix overflow
        mag_filtered -= Pi2;
    
    //printf("head: %.3f , %.3f   \t", mag_filtered, mag_unfiltered);
    //printf("mag: %.3f,    mFilt: %.3f   |  raw: %.3f, %.3f, %.3f    |  tilt: %.3f, %.3f, %.3f \t", mag_unfiltered,  mag_filtered,    mx_raw, my_raw, mz_raw,    tilt_mag_x, tilt_mag_y, tilt_mag_z);
    
    z_angle = (alpha * gz_filtered + (1.0 - alpha) * (mag_filtered-mag_filtered_last)) - (10 * y_joy);
    
    //recover from nan error
    if(x_angle != x_angle) x_angle = x_angle_last; 
    if(y_angle != y_angle) y_angle = y_angle_last;
    if(z_angle != z_angle) z_angle = z_angle_last;
    
    //boundry on joystick
    if(x_joy > 0.25) x_joy = 0.25;
    else if(x_joy < -0.25) x_joy = -0.25;
    if(y_joy > 0.25) y_joy = 0.25;
    else if(y_joy < -0.25) y_joy = -0.25;

    //remember last values for next cycle
    x_angle_last = x_angle;
    y_angle_last = y_angle;  
    z_angle_last = z_angle; 
    mag_filtered_last = mag_filtered;    
    
    //return values
    *x_angle_return = x_angle;
    *y_angle_return = y_angle;
    *z_angle_return = z_angle; 
}

//sets access between IMU and magnetometer
void IMU::setMasterMode(bool en) {
    if(en){
        conn.i2c_write_byte(IMU_ADDRESS, RA_USER_CTRL, 0x20);
    }
    else {
        conn.i2c_write_byte(IMU_ADDRESS, RA_USER_CTRL, 0x00);
    }
}

//sets access between IMU and magnetometer
void IMU::setI2CBypassMode(bool en){
    if(en){
        conn.i2c_write_byte(IMU_ADDRESS, INTERRUPT_PIN_CONFIG, 0x02);
    }
    else {
        conn.i2c_write_byte(IMU_ADDRESS, INTERRUPT_PIN_CONFIG, 0x00);   
    } 
}

//function to include magnetometers data in IMUs registers
void IMU::setSlaveAddress(uint8_t regOffset, char address){
    if(regOffset > 3) return;
    conn.i2c_write_byte(IMU_ADDRESS, RA_I2C_SLV0_ADDR + regOffset*3, address);
}

//function to include magnetometers data in IMUs registers
void IMU::setSlaveRegister(uint8_t regOffset, char reg){
    if(regOffset > 3) return;
    conn.i2c_write_byte(IMU_ADDRESS, RA_I2C_SLV0_REG + regOffset*3, reg);
}

//function to include magnetometers data in IMUs registers
void IMU::setSlaveControl(int regNo, char ctrl){
    conn.i2c_write_byte(IMU_ADDRESS, RA_I2C_SLV0_REG, regNo);
    conn.i2c_write_byte(IMU_ADDRESS, RA_I2C_SLV0_CTRL, ctrl);      
}

//tests magnetometer connection through i2c function
int IMU::testMagnetometerConnection(){
    return conn.testMagnetometerConnection(); 
}

//initilize magnetometer
void IMU::initializeMagnetometer(){
    //config register A
    conn.i2c_write_byte(MAG_ADDRESS, CONFIG_A, AVG8_SAMPLES | OUTPUT_RATE_75);
    //config registerB
    conn.i2c_write_byte(MAG_ADDRESS, CONFIG_B, 0x20);    
    //set mode 
    conn.i2c_write_byte(MAG_ADDRESS, MODE_REG, 0x00);
}

//gets raw magnetometer data 
void IMU::magRawData(){//uint16_t *mx, uint16_t *my, uint16_t *mz){    
    unsigned char buffer[6];
    
    conn.i2c_read(MAG_ADDRESS, RA_DATAX, 6, buffer);//0x18, 0x03, 6, buffer);
    //conn.i2c_read(IMU_ADDRESS, 0x49, 6, buffer);

    //fetch magnetometer raw data
    mx_raw = (( (int16_t)buffer[1]) << 8) | buffer[0];
    mz_raw = (( (int16_t)buffer[3]) << 8) | buffer[2];
    my_raw = (( (int16_t)buffer[5]) << 8) | buffer[4];
    
//    *mx = mx_raw;
//    *my = my_raw;
//    *mz = mz_raw;
}
    