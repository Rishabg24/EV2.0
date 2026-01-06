#include <Arduino.h>
#include <Encoder.h>

// =============================================================
// PID CLASS
// =============================================================
class PID {
private:
    float kp, ki, kd;
    float integralMax;
    float integral;
    float prevError;
    unsigned long prevTime;
    
public:
    PID(float p, float i, float d, float intMax = 100.0) {
        kp = p;
        ki = i;
        kd = d;
        integralMax = intMax;    
        reset();
    }

    void reset() {
        integral = 0;
        prevError = 0;
        prevTime = millis();
    }
        
    float compute(float setpoint, float measured) {
        unsigned long currentTime = millis();
        float dt = (currentTime - prevTime) / 1000.0;
        if (dt <= 0.0) return 0.0;

        float error = setpoint - measured;
        float P = kp * error;

        integral += error * dt;
        if(integral > integralMax) integral = integralMax;
        else if (integral < -integralMax) integral = -integralMax;

        float I = ki * integral;
        float D = kd * (error - prevError) / dt;

        prevError = error;
        prevTime = currentTime;

        return P + I + D;
    }
};

// =============================================================
// HARDWARE PINS & CONSTANTS
// =============================================================
#define ENA 6
#define IN1 7
#define IN2 8
#define ENB 9
#define IN3 10
#define IN4 11
#define BUTTON_PIN 12

Encoder leftEncoder(2, 4);
Encoder rightEncoder(5, 3);

bool isRunning = false;
bool lastButtonState = HIGH;

// Run Configuration
const float INTERVAL_DISTANCE = 2.0;  // Each interval is 2 meters
const int NUM_INTERVALS = 1;          // 4 intervals = 8 meters total
const float FINAL_DISTANCE = 0.75;    // Final 0.75 meter run

// Current run state
int currentInterval = 0;              // Which interval we're on (0-3)
bool inFinalRun = false;              // Are we in the 0.75m final run?

// Mechanical Specs
const float WHEEL_DIAMETER_CM = 4.5; 
const float COUNTS_PER_REVOLUTION = 1200.0;
const float MAX_RPM_AT_FULL_POWER = 268.67;

// Math Conversions
const float WHEEL_CIRCUMFERENCE_M = (WHEEL_DIAMETER_CM / 100.0) * PI;
const float COUNTS_PER_METER = COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_M;

long targetEncoderCount = 0;

// DUAL PID - One for each motor speed (6 constants total!)
// Left Motor PID Constants
const float LEFT_KP = 8.0;
const float LEFT_KI = 2.0;
const float LEFT_KD = 0.1;

// Right Motor PID Constants
const float RIGHT_KP = 8.0;
const float RIGHT_KI = 2.0;
const float RIGHT_KD = 0.1;

PID leftMotorPID(LEFT_KP, LEFT_KI, LEFT_KD, 50.0);
PID rightMotorPID(RIGHT_KP, RIGHT_KI, RIGHT_KD, 50.0);

// Speed tracking
long lastLeftCount = 0;
long lastRightCount = 0;
unsigned long lastSpeedUpdate = 0;
const unsigned long SPEED_UPDATE_INTERVAL = 50; // ms

float leftSpeed = 0;   // counts per second
float rightSpeed = 0;  // counts per second

// Target speed (we'll compute this from your max RPM)
float targetSpeed = 0;  // counts per second

int maxPWM = 255;
int minPWM = 70;

// =============================================================
// HELPER FUNCTIONS
// =============================================================

void setMotorSpeed(int leftPWM, int rightPWM) {
    if (leftPWM >= 0) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        analogWrite(ENA, constrain(leftPWM, 0, 255));
    } else {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
        analogWrite(ENA, constrain(abs(leftPWM), 0, 255));
    }
    
    if (rightPWM >= 0) {
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
        analogWrite(ENB, constrain(rightPWM, 0, 255));
    } else {
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        analogWrite(ENB, constrain(abs(rightPWM), 0, 255));
    }
}

void stopMotors() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void brakeMotors() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 255);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
    delay(100);
    stopMotors();
}

void resetForNextRun() {
    leftEncoder.write(0);
    rightEncoder.write(0);
    leftMotorPID.reset();
    rightMotorPID.reset();
    
    lastLeftCount = 0;
    lastRightCount = 0;
    leftSpeed = 0;
    rightSpeed = 0;
    lastSpeedUpdate = millis();
    
    // Calculate target speed: use 90% of max RPM for reliability
    float targetRPM = MAX_RPM_AT_FULL_POWER * 0.9;
    targetSpeed = (targetRPM / 60.0) * COUNTS_PER_REVOLUTION; // counts/sec
    
    if (!inFinalRun) {
        targetEncoderCount = (long)(INTERVAL_DISTANCE * COUNTS_PER_METER);
        Serial.print("Starting interval ");
        Serial.print(currentInterval + 1);
        Serial.print(" of ");
        Serial.println(NUM_INTERVALS);
    } else {
        targetEncoderCount = (long)(FINAL_DISTANCE * COUNTS_PER_METER);
        Serial.println("Starting FINAL 0.75m run");
    }
    
    Serial.print("Target speed: ");
    Serial.print(targetSpeed);
    Serial.println(" counts/sec");
}

void updateSpeeds() {
    unsigned long now = millis();
    if (now - lastSpeedUpdate >= SPEED_UPDATE_INTERVAL) {
        float dt = (now - lastSpeedUpdate) / 1000.0;
        
        long leftCount = leftEncoder.read();
        long rightCount = -rightEncoder.read();
        
        // Calculate speeds (counts per second)
        leftSpeed = (leftCount - lastLeftCount) / dt;
        rightSpeed = (rightCount - lastRightCount) / dt;
        
        lastLeftCount = leftCount;
        lastRightCount = rightCount;
        lastSpeedUpdate = now;
    }
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    stopMotors();
    Serial.begin(115200);
    delay(500);

    Serial.println("--- DUAL PID INTERVAL ROBOT ---");
    Serial.println("Run sequence:");
    Serial.println("  - 4 intervals of 2m each (total 8m)");
    Serial.println("  - Then 0.75m final run");
    Serial.println("Press Button to Start...");
}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {
    // 1. Button Handling
    bool buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && lastButtonState == HIGH) {
        delay(50);
        if (digitalRead(BUTTON_PIN) == LOW) {
            if (!isRunning) {
                isRunning = true;
                currentInterval = 0;
                inFinalRun = false;
                resetForNextRun();
                Serial.println("STARTING SEQUENCE...");
            } else {
                isRunning = false;
                stopMotors();
                Serial.println("EMERGENCY STOPPED");
            }
        }
    }
    lastButtonState = buttonState;
    
    // 2. Drive Logic
    if (isRunning) {
        // Update speed measurements
        updateSpeeds();
        
        long leftCount = leftEncoder.read();
        long rightCount = -rightEncoder.read();
        
        long currentDistance = abs(leftCount);
        long distanceRemaining = targetEncoderCount - currentDistance;

        // Check if we've reached the target
        if (distanceRemaining <= 0) {
            brakeMotors();
            delay(500);
            
            if (!inFinalRun) {
                currentInterval++;
                
                if (currentInterval >= NUM_INTERVALS) {
                    Serial.println("*** 8 METERS COMPLETE - Starting final run ***");
                    inFinalRun = true;
                    resetForNextRun();
                } else {
                    resetForNextRun();
                }
            } else {
                isRunning = false;
                stopMotors();
                Serial.println("*** SEQUENCE COMPLETE - 8.75m traveled ***");
                Serial.println("Press button to start new sequence.");
            }
        } 
        else {
            // Adjust target speed for deceleration near end
            float currentTargetSpeed = targetSpeed;
            long slowDownThreshold = 400;
            
            if (distanceRemaining < slowDownThreshold) {
                float ratio = (float)distanceRemaining / slowDownThreshold;
                currentTargetSpeed = targetSpeed * ratio;
                currentTargetSpeed = max(currentTargetSpeed, targetSpeed * 0.3); // min 30%
            }

            // Each motor gets PID control to match target speed
            float leftPWM = leftMotorPID.compute(currentTargetSpeed, leftSpeed);
            float rightPWM = rightMotorPID.compute(currentTargetSpeed, rightSpeed);
            
            // Constrain outputs
            leftPWM = constrain(leftPWM, minPWM, maxPWM);
            rightPWM = constrain(rightPWM, minPWM, maxPWM);
            
            setMotorSpeed((int)leftPWM, (int)rightPWM);
            
            // Debugging
            static unsigned long lastPrint = 0;
            if (millis() - lastPrint > 250) {
                Serial.print(inFinalRun ? "FINAL | " : "Int ");
                if (!inFinalRun) Serial.print(currentInterval + 1);
                Serial.print(" | Dist: "); Serial.print(currentDistance);
                Serial.print("/"); Serial.print(targetEncoderCount);
                Serial.print(" | L_spd: "); Serial.print(leftSpeed, 0);
                Serial.print(" | R_spd: "); Serial.print(rightSpeed, 0);
                Serial.print(" | L_PWM: "); Serial.print((int)leftPWM);
                Serial.print(" | R_PWM: "); Serial.println((int)rightPWM);
                lastPrint = millis();
            }
        }
    }

    delay(10);
}