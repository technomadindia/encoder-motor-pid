#include <Arduino.h>
#include <TimerOne.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// #define _DEBUG

#define SLEEP_PIN 2
#define ENCA_PIN 3
#define ENCB_PIN 4
#define M1_PIN 5
#define M2_PIN 6

// PID constants for N20 motor
const float KP = 1.0f;
const float KD = 0.1f;
const float KI = 0.05f;

// specify interrupt variable as volatile
volatile long g_positionInterrupt = 0L;
volatile bool timerTrigger = true;

// Encoder interrupt routine
void readEncoder() { // on rising edge of Encode A
    int encBState = digitalRead(ENCB_PIN);
    if (encBState > 0) { // if Encoder B is ahead implies CW rotation
        g_positionInterrupt++;
    } else { // if Encoder B is lagging implies CCW rotation
        g_positionInterrupt--;
    }
}

// set trigger for control loop update
void timerCallback() {
    timerTrigger = true;
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
        digitalWrite(motor1_pin, HIGH);
        digitalWrite(motor2_pin, HIGH);
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

// Timing loop state
static unsigned long prevTime = 0UL;

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

    // set initial time
    prevTime = micros();

    // setup timer2 for control loop update
    Timer1.initialize(1000); // 1 KHz
    Timer1.attachInterrupt(timerCallback);
}

void loop() {
    // only if time for control loop update
    if (true == timerTrigger) {
        // time difference
        unsigned long currTime = micros();
        float deltaTime = ((float)(currTime - prevTime)) / (1.0e6f);
        prevTime = currTime;

        // Read the position in an atomic block to avoid a potential
        // misread if the interrupt coincides with this code running
        long currentPosition = 0L;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            currentPosition = g_positionInterrupt;
        }

        // set target position
        long targetPosition = (long)(350.0f * (sin(currTime / 1e6f) > 0.0f));
        // int targetPosition = (long)(350.0f * sin(currTime / 1e6f));

        // update feedback control loop
        float controlSignal = pid_controller(targetPosition, currentPosition, deltaTime, KP, KD, KI);

        // motor direction based on sign of control signal
        int motorDirection = (controlSignal < 0.0f) ? -1 : 1;

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
        char buffer[256];
        sprintf(buffer, ">dt:%d\r\n>SP:%ld\r\n>PV:%ld\r\n>CS:%d\r\n>mpwr:%d\r\n",
                (int)(deltaTime * 1e6f), targetPosition, currentPosition, (int)controlSignal, motorPower);
        Serial.print(buffer);
#endif

        // rest trigger
        timerTrigger = false;
    }
}
