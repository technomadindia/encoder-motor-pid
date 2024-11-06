#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// #define _DEBUG

#define ENCA_PIN 2
#define ENCB_PIN 3
#define M1_PIN 5
#define M2_PIN 6

// PID constants for N20 motor
const float KP = 1.0;
const float KD = 0.1;
const float KI = 0.05;

// specify interrupt variable as volatile
volatile int g_positionInterrupt = 0;

void readEncoder();
float pid_controller(int desired, int measured, float deltaTime, float kp, float kd, float ki);
void setMotor(int direction, int pwmValue, int motor1_pin, int motor2_pin);

void setup() {
#if _DEBUG
    Serial.begin(9600);
#endif

    // set I/O configuration
    pinMode(ENCA_PIN, INPUT);
    pinMode(ENCB_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_PIN), readEncoder, RISING);

    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
}

// Timing loop state
long prevTime = 0;

void loop() {
    // set target position
    int targetPosition = 350 * sin(prevTime / 1e6);

    // time difference
    long currTime = micros();
    float deltaTime = ((float)(currTime - prevTime)) / (1.0e6);
    prevTime = currTime;

    // Read the position in an atomic block to avoid a potential
    // misread if the interrupt coincides with this code running
    int currentPosition = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currentPosition = g_positionInterrupt;
    }

    // update feedback control loop
    float controlSignal = pid_controller(targetPosition, currentPosition, deltaTime, KP, KD, KI);

    // motor power clipped to 8-bit PWM range
    int motorPower = (int)fabs(controlSignal);
    if (motorPower > 255) {
        motorPower = 255;
    }

    // motor direction based on sign of control signal
    int motorDirection = 1;
    if (controlSignal < 0) {
        motorDirection = -1;
    }

    // signal the motor
    // ps: 20% deadband
    setMotor((motorPower > 51) ? motorDirection : 0, motorPower, M1_PIN, M2_PIN);

#if _DEBUG
    // debug: monitoring
    Serial.print(">targetPosition:");
    Serial.println(targetPosition);
    // Serial.print(" ");
    Serial.print(">currentPosition:");
    Serial.println(currentPosition);
    // Serial.print(" ");
    Serial.print(">motorPower:");
    Serial.println(motorPower);
#endif
}

// Encoder interrupt routine
void readEncoder() { // on rising edge of Encode A
    int encBState = digitalRead(ENCB_PIN);
    if (encBState > 0) { // if Encoder B is ahead implies CW rotation
        g_positionInterrupt++;
    } else { // if Encoder B is lagging implies CCW rotation
        g_positionInterrupt--;
    }
}

// PID state variables
float previousError = 0;
float errorIntegral = 0;

// PID control feedback loop
float pid_controller(int desired, int measured, float deltaTime, float kp, float kd, float ki) {
    // error
    int error = desired - measured;

    // derivative
    float errorDerivative = (error - previousError) / (deltaTime);

    // integral
    errorIntegral += error * deltaTime;

    // control signal
    float controlSignal = kp * error + kd * errorDerivative + ki * errorIntegral;

    // store previous error
    previousError = error;

    return controlSignal;
}

// direction must be either of 1, 0, -1
// pwmValues must be [0, 255]
// motor1_pin and motor2_pin both must support pwm output
void setMotor(int direction, int pwmValue, int motor1_pin, int motor2_pin) {
    if (direction == 1) { // CW rotation
        analogWrite(motor1_pin, pwmValue);
        digitalWrite(motor2_pin, LOW);
    } else if (direction == -1) { // CCW rotation
        digitalWrite(motor1_pin, LOW);
        analogWrite(motor2_pin, pwmValue);
    } else { // braking
        digitalWrite(motor1_pin, LOW);
        digitalWrite(motor2_pin, LOW);
    }
}
