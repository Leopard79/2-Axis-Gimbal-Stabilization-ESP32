#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <ESP32Servo.h>

// ─── Pin Definitions ────────────────────────────────────────────────────────
#define PAN_PIN  2
#define TILT_PIN 15
#define INT_PIN  19   // MPU6050 interrupt pin

Servo tiltServo;
Servo panServo;
MPU6050 mpu;

// ─── PID Variables ──────────────────────────────────────────────────────────
double Setpoint = 0;
double inputPitch, outputPitch;
double inputRoll,  outputRoll;

// Tuned using Ziegler-Nichols method: Ku=1.6, Tu=0.21s
// Kp=0.6*Ku, Ki=1.2*Ku/Tu, Kd=0.075*Ku*Tu
double Kp = 0.8, Ki = 16.8, Kd = 0.0105;

PID PIDPitch(&inputPitch, &outputPitch, &Setpoint, Kp, Ki, Kd, REVERSE);
PID PIDRoll (&inputRoll,  &outputRoll,  &Setpoint, Kp, Ki, Kd, REVERSE);

// ─── MPU6050 DMP Variables ──────────────────────────────────────────────────
uint8_t  fifoBuffer[64];
uint16_t packetSize;
uint16_t fifoCount;
uint8_t  mpuIntStatus;
Quaternion  q;
VectorFloat gravity;
float ypr[3];  // [yaw, pitch, roll] in radians

// Interrupt flag — set by ISR when DMP data is ready
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(100000);

    tiltServo.attach(TILT_PIN);
    panServo.attach(PAN_PIN);
    tiltServo.write(90);
    panServo.write(90);
    delay(500);

    mpu.initialize();

    uint8_t status = mpu.dmpInitialize();
    if (status == 0) {
        // Auto-calibration — keep sensor flat and still during startup
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        packetSize   = mpu.dmpGetFIFOPacketSize();
        mpuInterrupt = false;
        Serial.println("DMP Ready");
    } else {
        Serial.print("DMP Init Failed: ");
        Serial.println(status);
        while (true);
    }

    PIDPitch.SetMode(AUTOMATIC);
    PIDPitch.SetSampleTime(10);
    PIDPitch.SetOutputLimits(-90, 90);
    PIDRoll.SetMode(AUTOMATIC);
    PIDRoll.SetSampleTime(10);
    PIDRoll.SetOutputLimits(-90, 90);
}

// ─── Main Loop ──────────────────────────────────────────────────────────────
void loop() {

    // Runtime PID tuning via Serial (from MATLAB GUI)
    // Format: 'P<value>', 'I<value>', 'D<value>', 'S<setpoint>'
    if (Serial.available() > 0) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();
        char   type = msg[0];
        double val  = msg.substring(1).toDouble();
        if      (type == 'P') Kp = val;
        else if (type == 'I') Ki = val;
        else if (type == 'D') Kd = val;
        else if (type == 'S') Setpoint = val;
        PIDPitch.SetTunings(Kp, Ki, Kd);
        PIDRoll.SetTunings(Kp, Ki, Kd);
    }

    // PID runs continuously between DMP interrupts for fast response
    while (!mpuInterrupt && fifoCount < packetSize) {
        PIDPitch.Compute();
        PIDRoll.Compute();
        tiltServo.write(constrain(90 + (int)outputPitch, 40, 140));
        panServo.write(constrain(90 + (int)outputRoll,   40, 140));
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount    = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        return;
    }

    if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        inputPitch = ypr[1] * 180.0 / M_PI;
        inputRoll  = ypr[2] * 180.0 / M_PI;

        Serial.println(inputPitch);
    }
}