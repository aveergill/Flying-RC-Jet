# Flying-RC-Jet
We will build a high-performance delta-wing RC jet with carbon fiber spars, powered by twin BLDC motors and a custom EDF system, featuring active thrust vectoring and Arduino-based PID gyro control. The project delivers a flying prototype, open-source firmware, and full build documentation.

/*
set gyro limits in line no  98,99
reverse channels gyro in line no  104,105
reverce rc inputs in line no  108,109
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>

// MPU6050
#define INTERRUPT_PIN 2  
MPU6050 accelgyro;
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// Servo definitions
Servo servoAileron, servoElevator;

// RC Receiver Inputs
byte rcpinAileron = 7;  // Aileron Channel (CH1)
byte rcpinElevator = 6; // Elevator Channel (CH2)
byte rcpinMode = 5;     // Mode Switch Channel (CH3)

// Variables to store RC PWM values
unsigned long pwmAileron, pwmElevator, pwmMode;
bool autoLevel = true;

// MPU control/status variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(38400);
    
    // Initialize MPU6050
    accelgyro.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    devStatus = accelgyro.dmpInitialize();
    accelgyro.setXGyroOffset(-203);
    accelgyro.setYGyroOffset(-95);
    accelgyro.setZGyroOffset(-48);
    accelgyro.setZAccelOffset(1933);
    
    if (devStatus == 0) {
        accelgyro.CalibrateAccel(6);
        accelgyro.CalibrateGyro(6);
        accelgyro.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        dmpReady = true;
        packetSize = accelgyro.dmpGetFIFOPacketSize();
    } else {
        Serial.println("DMP Initialization failed!");
    }
    
    // Attach servos
    servoAileron.attach(9);
    servoElevator.attach(8);
    
    // Set RC pins as input
    pinMode(rcpinAileron, INPUT);
    pinMode(rcpinElevator, INPUT);
    pinMode(rcpinMode, INPUT);
}

void loop() {
    if (!dmpReady) return;

    if (accelgyro.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        ypr[1] = ypr[1] * 180 / M_PI; // Pitch
        ypr[2] = ypr[2] * 180 / M_PI; // Roll
    }
    
    // Read RC PWM signals
    pwmAileron = pulseIn(rcpinAileron, HIGH);
    pwmElevator = pulseIn(rcpinElevator, HIGH);
    pwmMode = pulseIn(rcpinMode, HIGH);

    // Determine mode (Auto-Level ON/OFF based on RC switch position)
    autoLevel = (pwmMode > 1500); // Assuming switch high is auto-level, low is manual

    // Normalize RC inputs (1000-2000 μs to -35° to 35° range)
    int rcAileron = map(pwmAileron, 1000, 2000, -35, 35);       //CHANGE AILERON GYRO LIMITS BY INCREACING AND DECREASING -35, 35
    int rcElevator = map(pwmElevator, 1000, 2000, -30, 30);     //CHANGE ELEVATOR GYRO LIMITS BY INCREACING AND DECREASING -35, 35

    int rollValue, pitchValue;
    if (autoLevel) {
        // Blend gyro-based auto-leveling with RC input
        rollValue = constrain(map(ypr[2], 90, -90, 180, 0) + rcAileron, 0, 180);    //REVERSE AILERON GYRO BY SWAPING 90 AND -90 
        pitchValue = constrain(map(ypr[1], 90, -90, 180, 0) + rcElevator, 0, 180);  //REVERSE ELEVATOR GYRO BY SWAPING 90 AND -90 
    } else {
        // Manual mode, direct RC control
        rollValue = map(rcAileron, -45, 45, 0, 180);        //REVERCE AILERON RC INPUT BY REPLACING -45 AND 45
        pitchValue = map(rcElevator, -45, 45, 0, 180);      //REVERCE ELEVATOR RC INPUT BY REPLACING -45 AND 45
    }
    
    servoAileron.write(rollValue);
    servoElevator.write(pitchValue);
}
