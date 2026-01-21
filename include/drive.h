#ifndef DRIVE_H
#define DRIVE_H
#include <Arduino.h>
#include <Encoder.h>
#include "PID.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <kalman.h>

class Drive{
    public:
        
        Drive(PID& Lcon, PID& Rcon, Motor& Lmotor,Motor& Rmotor, Encoder& Lenc, Encoder& Renc, Adafruit_MPU6050& mpu6050, float wheelBase, float wheelDiameter): 
            Lcon(Lcon),
            Rcon(Rcon),
            Rmotor(Rmotor),
            Lmotor(Lmotor),
            Lenc(Lenc),
            Renc(Renc),
            mpu(mpu6050)                                                                                            
    {
        wBase = wheelBase;
        wDiameter = wheelDiameter;
    }
        void driveDistance(float distance, int speed);
        void stop();
        void reset();
        void turnR(int speed);
        void turnL(int speed);
        void sTurnR(int speed);
        void sTurnL(int speed);
        void turn(float degree, int speed);
        void begin();
        void accel(int targetSpeed, int accelRate);
        void decel(int targetSpeed, int decelRate);

    private:

        PID& Lcon;
        PID& Rcon;
        Motor& Rmotor;
        Motor& Lmotor;
        Encoder& Lenc;
        Encoder& Renc;
        Adafruit_MPU6050& mpu;


        unsigned long now;
        float wBase;
        float wDiameter;
        float bias = 0.f;

        float Kp = 0.0001f; // if this doesn't work try 30.0f

        uint8_t startPWM(int linSpeed);

        EKFState ekf(float wb, float wr);

};

#endif