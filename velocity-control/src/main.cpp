#include <Arduino.h>

#include <util/atomic.h>

#define _DEBUG

#define SLEEP_PIN 2
#define ENCA_PIN 3
#define ENCB_PIN 4
#define M1_PIN 5
#define M2_PIN 6

// PID constants for N20 motor
const float KP = 1.0;
const float KD = 0.01;
const float KI = 0.05;

// Timing loop state
long prevTime = 0;
int prevPosition = 0;

// specify interrupt variable as volatile
volatile int g_positionInterrupt = 0;
volatile float g_velocityInterrupt = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void readEncoder();
float pid_controller(int desired, int measured, float deltaTime, float kp, float kd, float ki);
void setMotor(int direction, int pwmValue, int motor1_pin, int motor2_pin);

void setup() {
    // put your setup code here, to run once:
#ifdef _DEBUG
    Serial.begin(115200);
#endif

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
    int currentPosition = 0;
    float velocity2 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currentPosition = g_positionInterrupt;
        velocity2 = g_velocityInterrupt;
    }

    // Compute velocity with method 1
    long currTime = micros();
    float deltaTime = ((float)(currTime - prevTime)) / 1.0e6;
    float velocity1 = ((float)(currentPosition - prevPosition)) / deltaTime;
    prevPosition = currentPosition;
    prevTime = currTime;

    // Convert count/s to RPM
    float v1 = velocity1 / 300.0 * 60.0;
    float v2 = velocity2 / 300.0 * 60.0;

    // Low-pass filter (25 Hz cutoff)
    v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
    v1Prev = v1;
    v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
    v2Prev = v2;

    // Set a target
    float vt = 255 * (sin(currTime / 1e6) > 0);

    // update feedback control loop
    float controlSignal = pid_controller(vt, v1Filt, deltaTime, KP, KD, KI);

    // motor direction based on sign of control signal
    int motorDirection = (controlSignal < 0) ? -1 : 1;

    // motor power clipped to 8-bit PWM range
    int motorPower = (int)fabs(controlSignal);
    if (motorPower >= 255) { // clamp to max PWM
        motorPower = 255;
    } else if (motorPower >= 51) { // linear zone
        motorPower = motorPower;
    } else if (motorPower >= 13) { // clamp to deadzone
        motorPower = 51;
    } else if (motorPower > 0) { // clamp to zero
        motorPower = 0;
        motorDirection = 0;
    }

    // signal the motor
    setMotor(motorDirection, motorPower, M1_PIN, M2_PIN);

#ifdef _DEBUG
    // debug: teleplot monitoring
    Serial.print(">vt:");
    Serial.println(vt);
    Serial.print(">v1Filt:");
    Serial.println(v1Filt);
    Serial.print(">cs:");
    Serial.println(controlSignal);
    Serial.print(">mp:");
    Serial.println(motorPower);
#endif

    delay(1);
}

// Encoder interrupt routine
void readEncoder() { // on rising edge of Encode A
    int encBState = digitalRead(ENCB_PIN);
    if (encBState > 0) { // if Encoder B is ahead implies CW rotation
        g_positionInterrupt++;
    } else { // if Encoder B is lagging implies CCW rotation
        g_positionInterrupt--;
    }

    // Compute velocity with method 2
    long currTime = micros();
    float deltaTime = ((float)(currTime - prevT_i)) / 1.0e6;
    g_velocityInterrupt = 1 / deltaTime;
    prevT_i = currTime;
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
