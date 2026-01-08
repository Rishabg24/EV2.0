#include "Drive.h"
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

void Drive::begin(){
    Serial.begin(9600);
    Rmotor.begin();
    Lmotor.begin();
    if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
        delay(10);
    }
    }
    Serial.println("MPU6050 Found!");
    // set accelerometer range to +-8G
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    // set gyro range to +- 500 deg/s
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    // set filter bandwidth to 21 Hz
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    sensors_event_t a , g, temp;
    mpu.getEvent(&a, &g, &temp);

    for(int i =0; i<1000; i++){
        bias += g.gyro.z;
        delay(1);
    }

    bias = bias/1000.f;
    Serial.print("Gyro Calibration done");

    delay(100);
}

void Drive::driveDistance(float distance, int speed){
    sensors_event_t a , g, temp;
    mpu.getEvent(&a, &g, &temp);
    float yaw = 0.f;
    float error = yaw;
    float gyroZ_corrected = 0.f;
    float steeringCorrection = 0.f;
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistance = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    while(abs(currDistance)<abs(distance)){
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.1f){
            Lmotor.drive(Lcon.output(Lenc,speed,dt));
            Rmotor.drive(Rcon.output(Renc,speed,dt));
            prevTime = now;
            currDistance = (Lenc.read()*Lcon.MPC+Renc.read()*Rcon.MPC)/2.f;
            Serial.print((now-start)/1000.f);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.println(currDistance);
        }
        gyroZ_corrected = g.gyro.z - bias;
        yaw += gyroZ_corrected*dt;
        error = yaw;
        steeringCorrection = Kp*error;
        now = millis();
    }
    stop();
}

void Drive::stop(){
    Rmotor.drive(0);
    Lmotor.drive(0);
}

void Drive::reset(){
    Renc.write(0);
    Lenc.write(0);
}

uint8_t Drive::startPWM(int linSpeed){
    float outputI = 40.f-40.f*log(1-abs(linSpeed)/550.f);
    uint8_t output = constrain((int)outputI, 40,230);
    return output;
}

void Drive::turnL(int speed){
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistanceLeft = 0.f;
    float currDistanceRight = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    while(currDistanceLeft < (250+wBase/2.f)*(PI/2.f) && currDistanceRight < (250-wBase/2.f)*(PI/2.f)){
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.02f){
            Lmotor.drive(Lcon.output(Lenc,speed*1.54,dt));
            Rmotor.drive(Rcon.output(Renc,speed,dt));
            prevTime = now;
            currDistanceLeft = Lenc.read()*Lcon.MPC;
            currDistanceRight = Renc.read()*Rcon.MPC;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed*1.54f);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(currDistanceLeft);Serial.print(",");
            Serial.println(currDistanceRight);
        }
        now = millis();
    }
    stop();
}

void Drive::turnR(int speed){
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistanceLeft = 0.f;
    float currDistanceRight = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    while(currDistanceLeft < (250-wBase/2.f)*(PI/2.f) && currDistanceRight < (250+wBase/2.f)*(PI/2.f)){
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.02f){
            Lmotor.drive(Lcon.output(Lenc,speed*1.54f,dt));
            Rmotor.drive(Rcon.output(Renc,speed,dt));
            prevTime = now;
            currDistanceLeft = Lenc.read()*Lcon.MPC;
            currDistanceRight = Renc.read()*Rcon.MPC;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(speed*1.54f);Serial.print(",");
            Serial.print(currDistanceLeft);Serial.print(",");
            Serial.println(currDistanceRight);
        }
        now = millis();
    }
    stop();
}

void Drive::sTurnL(int speed){
    sensors_event_t a , g, temp;
    mpu.getEvent(&a, &g, &temp);
    float heading = 0.f;
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;

    while(abs(heading)+4*PI/180.f<PI/2){
        mpu.getEvent(&a, &g, &temp);
        dt = (now-prevTime)/1000.f;
        if(abs(heading) > (PI/2.f)*0.8){
            speed = constrain(speed/2,50,speed);
        }
        if(dt>= 0.02f){
            Lmotor.drive(Lcon.output(Lenc,speed*-1,dt));
            Rmotor.drive(Rcon.output(Renc,speed,dt));
            prevTime = now;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(speed*-1);Serial.print(",");
            Serial.println(heading);
        }
        now = millis();
        heading += (g.gyro.z-bias)*dt;
    }
    stop();
}

void Drive::sTurnR(int speed){
    sensors_event_t a , g, temp;
    mpu.getEvent(&a, &g, &temp);
    float heading = 0.f;
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;

    while(abs(heading)+4*PI/180.f<PI/2){
        mpu.getEvent(&a, &g, &temp);
        dt = (now-prevTime)/1000.f;
        if(abs(heading) > (PI/2.f)*0.8){
            speed = constrain(speed/2,50,speed);
        }
        if(dt>= 0.02f){
            Lmotor.drive(Lcon.output(Lenc,speed,dt));
            Rmotor.drive(Rcon.output(Renc,speed*-1,dt));
            prevTime = now;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(speed*-1);Serial.print(",");
            Serial.println(heading);
        }
        now = millis();
        heading += (g.gyro.z-bias)*dt;
    }
    stop();
}

void Drive::turn(float degree, int speed){
    sensors_event_t a , g, temp;
    mpu.getEvent(&a, &g, &temp);
    float heading = 0.f;
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    if(degree<0){
        speed = -1*speed;
    }

    while(abs(heading)+4/90 *degree*PI/180.f<degree*PI/180.f){
        mpu.getEvent(&a, &g, &temp);
        now = millis();
        dt = (now-prevTime)/1000.f;
        if(abs(heading) > degree*(PI/180.f)*0.8){
            speed = constrain(speed/2,50,speed);
        }
        if(dt>= 0.02f){
            Lmotor.drive(Lcon.output(Lenc,speed,dt));
            Rmotor.drive(Rcon.output(Renc,speed*-1,dt));
            prevTime = now;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(speed*-1);Serial.print(",");
            Serial.println(heading);
        }
        heading += (g.gyro.z-bias)*dt;
    }
    stop();
}

void Drive::accel(int targetSpeed, int accelRate){
    int tspeed = 0;
    unsigned long start = millis();
    unsigned long prevTime = millis();
    unsigned long now = millis();
    float dt = (now-prevTime)/1000.f;
    float adt = (now-prevTime)/1000.f;
    int conAccel = accelRate;
    sensors_event_t a , g, temp;
    mpu.getEvent(&a, &g, &temp);
    while(Lcon.aSpeed < targetSpeed && Rcon.aSpeed < targetSpeed){
        now = millis();
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.02f){
            if(adt>= 0.6f){
                conAccel = accelRate;
                adt = 0.f;
            }else{
                conAccel = 0;
            }
            Lmotor.drive(Lcon.output(Lenc,tspeed,dt));
            Rmotor.drive(Rcon.output(Renc,tspeed,dt));
            prevTime = now;
            tspeed += accelRate*dt;
            Serial.print((now-start)/1000.f);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.println(tspeed);
        }
    }
}