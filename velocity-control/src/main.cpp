#include <Arduino.h>

#include <util/atomic.h>

#define _DEBUG

#define SLEEP_PIN 2
#define ENCA_PIN 3
#define ENCB_PIN 4
#define M1_PIN 5
#define M2_PIN 6

// PID constants for N20 motor
const float KP = 1.00f;
const float KD = 0.01f;
const float KI = 0.05f;

// conversion factor to encoder velocity to motor power
float KV = 0.0f;

// Timing loop state
long prevTime = 0L;
long prevPosition = 0L;

// specify interrupt variable as volatile
volatile long g_positionInterrupt = 0L;

float v1Filt = 0.0f;
float v1Prev = 0.0f;

void readEncoder();
float pid_controller(float desired, float measured, float deltaTime, float kp, float kd, float ki);
void setMotor(int direction, int pwmValue, int motor1_pin, int motor2_pin);

void setup() {
    // put your setup code here, to run once:
#ifdef _DEBUG
    Serial.begin(115200);
#endif

    // motor specs
    float rpm = 300;
    int gearRatio = 100;
    int encoderRate = 7;

    // velocity conversion factor
    KV = (255.0f * 60.0f) / (encoderRate * gearRatio * rpm);

    // set I/O configuration
    pinMode(ENCA_PIN, INPUT);
    pinMode(ENCB_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA_PIN), readEncoder, RISING);
    pinMode(SLEEP_PIN, OUTPUT);
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
}

void loop() {

    // Read the position in an atomic block to avoid a potential
    // misread if the interrupt coincides with this code running
    long currentPosition = 0L;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currentPosition = g_positionInterrupt;
    }

    // Compute velocity with method 1
    long currTime = micros();
    float deltaTime = ((float)(currTime - prevTime)) / 1.0e6f;
    float velocity1 = ((float)(currentPosition - prevPosition)) / deltaTime;
    prevPosition = currentPosition;
    prevTime = currTime;

    // Convert count/s to RPM
    float v1 = velocity1 * KV;

    // Low-pass filter (25 Hz cutoff)
    v1Filt = 0.8544f * v1Filt + 0.0728f * v1 + 0.0728f * v1Prev;
    v1Prev = v1;

    // Set a target
    float vt = 150 * (sin(currTime / 1e6) > 0); // 150.0f * sin(currTime / 1e6f) + 150.0f;

    // update feedback control loop
    float controlSignal = pid_controller(vt, v1Filt, deltaTime, KP, KD, KI);

    // motor direction based on sign of control signal
    int motorDirection = (controlSignal < 0.0f) ? -1 : 1;

    // motor power clipped to 8-bit PWM range
    int motorPower = (int)fabs(controlSignal);
    if (motorPower >= 255) { // clamp to max PWM
        motorPower = 255;
    } else if (motorPower < 51) {
        motorPower = 0;
    }

    // signal the motor
    setMotor(motorDirection, motorPower, M1_PIN, M2_PIN);

#ifdef _DEBUG
    // debug: teleplot monitoring
    Serial.print(">vtgt:");
    Serial.println(vt);
    Serial.print(">v1:");
    Serial.println(v1);
    Serial.print(">v1Fil:");
    Serial.println(v1Filt);
    Serial.print(">csig:");
    Serial.println(controlSignal);
    Serial.print(">mpwr:");
    Serial.println(motorPower);
    Serial.print(">cpos:");
    Serial.println(currentPosition);
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
float previousError = 0.0f;
float errorIntegral = 0.0f;

// PID control feedback loop
float pid_controller(float desired, float measured, float deltaTime, float kp, float kd, float ki) {
    // error
    float error = desired - measured;

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
    if (0 == direction) {              // free wheeling
        digitalWrite(SLEEP_PIN, LOW);  // sleep
    } else if (0 == pwmValue) {        // braking
        digitalWrite(SLEEP_PIN, HIGH); // wake
        digitalWrite(motor1_pin, LOW);
        digitalWrite(motor2_pin, LOW);
    } else if (1 == direction) {       // CW rotation
        digitalWrite(SLEEP_PIN, HIGH); // wake
        analogWrite(motor1_pin, pwmValue);
        digitalWrite(motor2_pin, LOW);
    } else if (-1 == direction) {      // CCW rotation
        digitalWrite(SLEEP_PIN, HIGH); // wake
        digitalWrite(motor1_pin, LOW);
        analogWrite(motor2_pin, pwmValue);
    }
}
