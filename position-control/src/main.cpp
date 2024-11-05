#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

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

long prevTime = 0;

void readEncoder();
float pid_controller(int desired, int measured, float deltaTime, float kp, float kd, float ki);
void setMotor(int direction, int pwmValue, int motor1_pin, int motor2_pin);

void setup() {
    Serial.begin(9600);
    pinMode(ENCA_PIN, INPUT);
    pinMode(ENCB_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_PIN), readEncoder, RISING);

    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);

    Serial.println("targetPosition currentPosition motorPower");
}

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

    float controlSignal = pid_controller(targetPosition, currentPosition, deltaTime, KP, KD, KI);

    // motor power clipped to 8-bit PWM range
    float motorPower = fabs(controlSignal);
    if (motorPower > 255) {
        motorPower = 255;
    }

    // motor direction based on sign of control signal
    int motorDirection = 1;
    if (controlSignal < 0) {
        motorDirection = -1;
    }

    // signal the motor
    setMotor((motorPower > 51) ? motorDirection : 0, motorPower, M1_PIN, M2_PIN);

    Serial.print(targetPosition);
    Serial.print(" ");
    Serial.print(currentPosition);
    Serial.print(" ");
    Serial.print(motorPower);
    Serial.println();
}

void readEncoder() {
    int encBState = digitalRead(ENCB_PIN);
    if (encBState > 0) {
        g_positionInterrupt++;
    } else {
        g_positionInterrupt--;
    }
}

// PID state variables
float previousError = 0;
float errorIntegral = 0;

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
