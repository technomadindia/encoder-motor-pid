#include <Arduino.h>

#include <util/atomic.h>

// Pins
#define ENCA 3
#define ENCB 4
#define IN1 5
#define IN2 6

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void readEncoder();
void setMotor(int direction, int pwmValue, int motor1_pin, int motor2_pin);

void setup() {
    Serial.begin(115200);

    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCA),
                    readEncoder, RISING);
}

void loop() {

    // read the position in an atomic block
    // to avoid potential misreads
    int pos = 0;
    float velocity2 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos = pos_i;
        velocity2 = velocity_i;
    }

    // Compute velocity with method 1
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    float velocity1 = (pos - posPrev) / deltaT;
    posPrev = pos;
    prevT = currT;

    // Convert count/s to RPM
    float v1 = velocity1 / 600.0 * 60.0;
    float v2 = velocity2 / 600.0 * 60.0;

    // Low-pass filter (25 Hz cutoff)
    v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
    v1Prev = v1;
    v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
    v2Prev = v2;

    // Set a target
    float vt = 100 * (sin(currT / 1e6) > 0);

    // Compute the control signal u
    float kp = 5;
    float ki = 10;
    float e = vt - v1Filt;
    eintegral = eintegral + e * deltaT;

    float u = kp * e + ki * eintegral;

    // Set the motor speed and direction
    int dir = 1;
    if (u < 0) {
        dir = -1;
    }
    int pwr = (int)fabs(u);
    if (pwr > 255) {
        pwr = 255;
    }
    setMotor(dir, pwr, IN1, IN2);

    Serial.print(vt);
    Serial.print(" ");
    Serial.print(v1Filt);
    Serial.println();
    delay(1);
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

void readEncoder() {
    // Read encoder B when ENCA rises
    int b = digitalRead(ENCB);
    int increment = 0;
    if (b > 0) {
        // If B is high, increment forward
        increment = 1;
    } else {
        // Otherwise, increment backward
        increment = -1;
    }
    pos_i = pos_i + increment;

    // Compute velocity with method 2
    long currT = micros();
    float deltaT = ((float)(currT - prevT_i)) / 1.0e6;
    velocity_i = increment / deltaT;
    prevT_i = currT;
}
