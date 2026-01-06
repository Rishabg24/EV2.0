// #include <Arduino.h>
// #include <Encoder.h>

// // =============================================================
// // PID CLASS
// // =============================================================
// class PID {
// private:
//     float kp, ki, kd;
//     float integralMax;
//     float integral;
//     float prevError;
//     unsigned long prevTime;
    
// public:
//     PID(float p, float i, float d, float intMax = 100.0) {
//         kp = p;
//         ki = i;
//         kd = d;
//         integralMax = intMax;    
//         reset();
//     }

//     void reset() {
//         integral = 0;
//         prevError = 0;
//         prevTime = millis();
//     }
        
//     float compute(float setpoint, float measured) {
//         unsigned long currentTime = millis();
//         float dt = (currentTime - prevTime) / 1000.0;
//         if (dt <= 0.0) return 0.0; // Prevent divide by zero

//         float error = setpoint - measured;
//         float P = kp * error;

//         integral += error * dt;
//         if(integral > integralMax) integral = integralMax;
//         else if (integral < -integralMax) integral = -integralMax;

//         float I = ki * integral;
//         float D = kd * (error - prevError) / dt;

//         prevError = error;
//         prevTime = currentTime;

//         return P + I + D;
//     }
// };

// // =============================================================
// // HARDWARE PINS & CONSTANTS
// // =============================================================
// #define ENA 6
// #define IN1 7
// #define IN2 8
// #define ENB 9
// #define IN3 10
// #define IN4 11
// #define BUTTON_PIN 12

// Encoder leftEncoder(2, 4);
// Encoder rightEncoder(5, 3);

// bool isRunning = false;
// bool lastButtonState = HIGH;

// // Target settings
// float Target_distance = 2.0; // meters
// float TARGET_TIME_SEC = 10.0; // Target time to cover distance

// // Mechanical Specs
// const float WHEEL_DIAMETER_CM = 4.5; 
// const float COUNTS_PER_REVOLUTION = 1200.0; // 12 CPR * 100:1 Gearbox
// const float MAX_RPM_AT_FULL_POWER = 268.67; // <--- CHECK YOUR MOTOR SPECS! 

// // Math Conversions
// const float WHEEL_CIRCUMFERENCE_M = (WHEEL_DIAMETER_CM / 100.0) * PI;
// const float COUNTS_PER_METER = COUNTS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE_M;

// long targetEncoderCount = 0;

// // PID
// PID straightPID(1.5, 0.0, 0.0);  

// int initialBaseSpeed = 0; // Will be calculated in setup
// int minSpeed = 70;        // Minimum PWM to keep motors moving
// float ENCODER_SCALE = 1.06; // 1.06 good for 2m

// // =============================================================
// // HELPER FUNCTIONS
// // =============================================================

// // Corrected Speed Calculation
// int calculateSpeed(float distanceM, float timeS) {
//     if (timeS <= 0) return 0;
    
//     // 1. Required Velocity (m/s)
//     float requiredSpeedMPS = distanceM / timeS; 
    
//     // 2. Required Wheel RPM
//     // v = (RPM * Circ) / 60  ->  RPM = (v * 60) / Circ
//     float requiredRPM = (requiredSpeedMPS * 60.0) / WHEEL_CIRCUMFERENCE_M; 

//     Serial.print("Target Speed (m/s): "); Serial.println(requiredSpeedMPS);
//     Serial.print("Target RPM: "); Serial.println(requiredRPM);

//     // 3. Map RPM to PWM (0-255)
//     // Fraction of max speed = requiredRPM / MAX_RPM
//     float pwmFloat = (requiredRPM / MAX_RPM_AT_FULL_POWER) * 255.0;
    
//     int pwmValue = (int)pwmFloat;
    
//     // Safety checks
//     if (pwmValue > 255) {
//         Serial.println("WARNING: Target speed exceeds motor capability! Caps at 255.");
//         return 255;
//     }
//     // Don't let it go below minSpeed or it won't move
//     if (pwmValue < minSpeed) {
//         Serial.println("WARNING: Target speed too slow, increasing to minSpeed.");
//         return minSpeed;
//     }
    
//     return pwmValue;
// }
// void setMotorSpeed(int leftSpeed, int rightSpeed) {
//     if (leftSpeed >= 0) {
//         digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
//         analogWrite(ENA, constrain(leftSpeed, 0, 255));
//     } else {
//         digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
//         analogWrite(ENA, constrain(abs(leftSpeed), 0, 255));
//     }
    
//     if (rightSpeed >= 0) {
//         digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
//         analogWrite(ENB, constrain(rightSpeed, 0, 255));
//     } else {
//         digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
//         analogWrite(ENB, constrain(abs(rightSpeed), 0, 255));
//     }
// }

// void stopMotors() {
//     digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
//     digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
//     analogWrite(ENA, 0); analogWrite(ENB, 0);
// }

// void brakeMotors() {
//     digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
//     analogWrite(ENA, 255); // Short circuit brake
//     digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
//     analogWrite(ENB, 255); // Short circuit brake
//     delay(100);
//     stopMotors();
// }

// // =============================================================
// // CALIBRATION FUNCTION
// // =============================================================
// void calibrateMaxRpm() {
//     Serial.println("\n--- STARTING RPM CALIBRATION ---");
    
//     // 1. Reset everything
//     leftEncoder.write(0);
//     rightEncoder.write(0);
//     stopMotors();
//     delay(500);

//     // 2. Drive at FULL POWER for a set duration
//     Serial.println("Running motors at PWM 255 for 1.0 second...");
//     digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Forward
//     digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Forward
    
//     unsigned long startTime = millis();
//     analogWrite(ENA, 255);
//     analogWrite(ENB, 255);
    
//     // Run for exactly 1000ms
//     while (millis() - startTime < 1000) {
//         // Do nothing, just let it run
//     }
    
//     stopMotors(); // Stop immediately after duration
    
//     // 3. Read final encoder values
//     long leftCount = leftEncoder.read();
//     long rightCount = -rightEncoder.read(); // Remember to negate
    
//     // 4. Calculate RPM (Revolutions Per Minute)
//     // Counts per second = Counts / 1.0 sec
//     // Revolutions per second = Counts / COUNTS_PER_REVOLUTION
//     // RPM = (Revolutions per second) * 60
    
//     float leftRPS = (float)leftCount / COUNTS_PER_REVOLUTION;
//     float rightRPS = (float)rightCount / COUNTS_PER_REVOLUTION;
    
//     float leftRPM = leftRPS * 60.0;
//     float rightRPM = rightRPS * 60.0;

//     Serial.println("--- CALIBRATION RESULTS ---");
//     Serial.print("Left Counts (1s): "); Serial.println(leftCount);
//     Serial.print("Right Counts (1s): "); Serial.println(rightCount);
//     Serial.print("Calculated Left RPM: "); Serial.println(leftRPM, 2);
//     Serial.print("Calculated Right RPM: "); Serial.println(rightRPM, 2);
    
//     // Use the AVERAGE of the two RPMs for the base calculation
//     float averageMaxRPM = (leftRPM + rightRPM) / 2.0;

//     Serial.print("\n>>> Use this value for MAX_RPM_AT_FULL_POWER: ");
//     Serial.println(averageMaxRPM, 2);
//     Serial.println("------------------------------------");
// }



// // =============================================================
// // SETUP
// // =============================================================
// void setup() {
//     pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
//     pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
//     pinMode(BUTTON_PIN, INPUT_PULLUP);
    
//     stopMotors();
//     Serial.begin(115200);
//     delay(500);

//     // --- NEW LINE ADDED FOR CALIBRATION ---
//     //calibrateMaxRpm(); 
//     // --------------------------------------

//     Serial.println("--- SYSTEM INIT ---");

//     // Calculate the base speed based on time target
//     initialBaseSpeed = 255; 
//     Serial.print("Calculated Base PWM: "); Serial.println(initialBaseSpeed);

//     Serial.println("Press Button to Start...");
// }

// // =============================================================
// // MAIN LOOP
// // =============================================================
// void loop() {
//     // 1. Button Handling
//     bool buttonState = digitalRead(BUTTON_PIN);
//     if (buttonState == LOW && lastButtonState == HIGH) {
//         delay(50);
//         if (digitalRead(BUTTON_PIN) == LOW) {
//             isRunning = !isRunning;
//             if (isRunning) {
//                 leftEncoder.write(0);
//                 rightEncoder.write(0);
//                 straightPID.reset();
//                 targetEncoderCount = (long)(Target_distance * COUNTS_PER_METER);
//                 Serial.println("STARTING...");
//             } else {
//                 stopMotors();
//                 Serial.println("STOPPED");
//             }
//         }
//     }
//     lastButtonState = buttonState;
    
//     // 2. Drive Logic
//     for(int i = 0; i<4;i++){

    
//     if (isRunning) {
//         long leftCount = leftEncoder.read();
//         long rightCount = -rightEncoder.read(); 
        
//         long currentDistance = abs(leftCount);
//         long distanceRemaining = targetEncoderCount - currentDistance;

//         // --- BRAKING LOGIC ---
//         if (distanceRemaining <= 0) {
//             brakeMotors(); 
//             isRunning = false; 
//             Serial.println("TARGET REACHED.");
//         } 
//         else {
//             // --- DECELERATION LOGIC ---
//             long slowDownThreshold = 1500; // approx 25cm
//             int currentSpeed = initialBaseSpeed;

//             if (distanceRemaining < slowDownThreshold) {
//                 // Map from currentBaseSpeed down to minSpeed
//                 currentSpeed = map(distanceRemaining, 0, slowDownThreshold, minSpeed, initialBaseSpeed);
//             }

//             // --- PID ---
//             float scaledRightCount = rightCount * ENCODER_SCALE;
//             float error = scaledRightCount - leftCount;
//             float correction = straightPID.compute(0, error);
            
//             // Limit correction
//             correction = constrain(correction, -120, 120);
            
//             int leftSpeed = currentSpeed - correction;
//             int rightSpeed = currentSpeed + correction;
            
//             setMotorSpeed(leftSpeed, rightSpeed);
            
//             // Debugging
//             static unsigned long lastPrint = 0;
//             if (millis() - lastPrint > 250) {
//                 Serial.print("Dist Rem: "); Serial.print(distanceRemaining);
//                 Serial.print(" | PWM: "); Serial.print(currentSpeed);
//                 Serial.print(" | Err: "); Serial.println(error);
//                 lastPrint = millis();
//             }
//         }
//     }
// }


//     delay(10);
// }