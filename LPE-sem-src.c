/*
 * Copyright 2020, Ondřej Masopust
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "mbed.h"

#define READ_BYTES 2

DigitalOut myled(PA_4);
I2C i2c(PF_0, PF_1);
PwmOut pwm(PB_1);
volatile double pwmDuty = 1000;
volatile int64_t angle = 0;
volatile int64_t integral = 0;
int16_t gyroOut = 0;


void posServo(void){
    /* 
     * This is the interrupt service routine which is called periodically to
     * compute the PWM signal sent to the servo.
     */
    double proportional = double(angle)/7611.0;
    integral += 2.0*double(angle)/7611.0;
    double derivative = double(gyroOut)/3500.0;
    // anti-windup
    integral = integral < 500 ? integral : 500;
    integral = integral > -500 ? integral : -500;
    pwmDuty = 1500 - (proportional + integral + derivative);
    // output saturation
    if(pwmDuty > 2000){
        pwmDuty = 2000;
        myled = 1; // indicate saturation
    } else if(pwmDuty < 1000){
        pwmDuty = 1000;
        myled = 1; // indicate saturation
    } else {
        myled = 0; // indicate normal mode
    }
    pwm.pulsewidth_us(pwmDuty);
}

void initSensor(const int add){
    /*
     * This function just wraps the sensor setup commands
     */
    // first element is the register address, second is the data to be written
    char config_setup[] = {0x1A, 0b00000110};
    char smplrt_div[] = {0x19, 0};
    char gyro_fs[] = {0x1B, 0b00011000};
    char cksel[] = {0x6B, 0b00001001};
    char intr_setup[] = {0x37, 0b00010000, 0x1};
    
    // set up the sensor digital low pass filter
    i2c.write(add, config_setup, 2, false);
    // set up the sensor sample rate
    i2c.write(add, smplrt_div, 2, false);
    // set up the gyro sensitivity
    i2c.write(add, gyro_fs, 2, false);
    // set the clock source to be X axis gyroscope
    i2c.write(add, cksel, 2, false);
    // set up the interrupt to occur whenever new data is ready
    i2c.write(add, intr_setup, 3, false);
}

int main() {
    const int add = 0b11010000; // sensor address
    i2c.frequency(400000);
    initSensor(add);
    
    const char zGyroAdd = 0x47; // address of the Z axis sensor
    int16_t offset = 0;

    pwm.period_ms(20); // 50 Hz
    pwm.pulsewidth_us(1500);
    wait_ms(1000);
    
    myled = 1; // indicate initializing offset
    
    // init the offset
    uint8_t testLen = 150;
    char read_data[READ_BYTES];
    int16_t value = 0;
    for(int i = 0; i < testLen; i++){
        i2c.write(add, &zGyroAdd, 1, true);
        i2c.read(add, read_data, READ_BYTES, false);
        value = int16_t((read_data[0] << 8)|(read_data[1]));
        offset += value;
        wait_ms(50);
    }
    offset /= testLen;

    // set up the interrupt
    Ticker servoUpdater;
    servoUpdater.attach(&posServo, 0.05);

    myled = 0; // indicate initialization done
    
    while(1) {
        // poll new data
        i2c.write(add, &zGyroAdd, 1, true);
        i2c.read(add, read_data, READ_BYTES, false);
        gyroOut = int16_t((read_data[0] << 8)|(read_data[1]));
        gyroOut -= offset;
        gyroOut = abs(gyroOut) > 45 ? gyroOut : 0; // remove noise
        angle += gyroOut; // integrate to get angle from angular velocity
        wait_us(900);
    }
}