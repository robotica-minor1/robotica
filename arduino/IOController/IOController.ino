// NOTE: Commands can have a maximum length of 127 characters and are not
// sanitised, use with care.

// Commands (>> = rpi to arduino, << = arduino to rpi):
//
// >> servos <ang1> <ang2> <ang3> <ang4> <speed1> <speed2> <speed3> <speed4> (degrees, +/- 40)
// >> props <prop1> <prop2> <prop3> <prop4> (thrust in micronewton tussen 0 en 11,000)
// >> propsraw <prop1> <prop2> <prop3> <prop4> (raw PWM between 800 and 1700)
// >> retracts <0/1> (1 = up)
// >> pollsonar
// >> shutdown (shuts down props, servos and retracts)
//
// << sonar <dist> (cm)
// << ack (sent if setter command successful, pollsonar will respond with data instead)
// << error (sent if command is wrong, other errors cause undefined behaviour)
//
// TODO: Uitzoeken waarom geen commando sturen gelijk is aan vorige commando

#include <VarSpeedServo.h>
#include <NewPing.h>
#include <EEPROM.h>
#include <Wire.h>

#include "Kalman.h"

// Configuration
const int PIN_ESC1 = 3;
const int PIN_ESC2 = 2;
const int PIN_ESC3 = 4;
const int PIN_ESC4 = 5;

const int PIN_SERVO1 = 13;
const int PIN_SERVO2 = 12;
const int PIN_SERVO3 = 11;
const int PIN_SERVO4 = 10;

const int PIN_US_TRIGGER = 52;
const int PIN_US_ECHO = 53;

const int PIN_RETRACTS = 48;

const int SONAR_MAX_DIST = 200;

const long MILLIS_PER_PROP_STEP = 100; // ms per PWM step of variable below (or less)
const long PWM_PER_PROP_STEP = 20; // PWM per step (or less)

// Globals
NewPing sonar(PIN_US_TRIGGER, PIN_US_ECHO, SONAR_MAX_DIST);

// Servo-like vars
VarSpeedServo retracts;
VarSpeedServo esc1, esc2, esc3, esc4;
VarSpeedServo servo1, servo2, servo3, servo4;

int angle1_mem = 0, angle2_mem = 0, angle3_mem = 0, angle4_mem = 0;

int minSpeed = 0;
int startSpeed = 30;
int maxSpeed = 127;

int minAngle = 0;
int maxAngle = 90;
int minPulse = 1000;
int maxPulse = 2000;

int alpha = 20;
int beta = 70;

int offset1 = -7;
int offset2 = 11;
int offset3 = -7;
int offset4 = -8;

int prop1_pwm = 0;
int prop1_target_pwm = 0;
int prop2_pwm = 0;
int prop2_target_pwm = 0;
int prop3_pwm = 0;
int prop3_target_pwm = 0;
int prop4_pwm = 0;
int prop4_target_pwm = 0;

long last_prop_step = 0;

bool shutdown = false;

bool prop1_detached = false;
bool prop2_detached = false;
bool prop3_detached = false;
bool prop4_detached = false;

double voltage_correct_factor = 1.0915;

// Motoren: 900 = 2000 RPM = -0.7 N, 1700 = 10600 RPM = 9.95 N
//
// Formules:
//
// rpm = microsec * 10.75 - 7675
// microsec = (rpm + 7675) / 10.75
// thrust = 0.001238372 * rpm - 3.176744
// rpm = (thrust + 3.176744) / 0.001238372
//
// Dus: 
int thrust2microseconds(float thrust) {
    float rpm = (thrust + 3.176744f) / 0.001238372f;
    return (rpm + 7675) / 10.75f;
}

void setup() {
    Serial.begin(115200);

    // Init retracts + escs
    retracts.attach(PIN_RETRACTS);

    esc1.attach(PIN_ESC1);
    esc2.attach(PIN_ESC2);
    esc3.attach(PIN_ESC3);
    esc4.attach(PIN_ESC4);

    esc1.writeMicroseconds(800);
    esc2.writeMicroseconds(800);
    esc3.writeMicroseconds(800);
    esc4.writeMicroseconds(800);

    prop1_target_pwm = prop1_pwm = 800;
    prop2_target_pwm = prop2_pwm = 800;
    prop3_target_pwm = prop3_pwm = 800;
    prop4_target_pwm = prop4_pwm = 800;

    last_prop_step = millis();

    // Init servos
    servo1.write(minPulse+((minPulse/(maxAngle))*beta), startSpeed);
    servo2.write(minPulse+((minPulse/(maxAngle))*alpha), startSpeed);
    servo3.write(minPulse+((minPulse/(maxAngle))*beta), startSpeed);
    servo4.write(minPulse+((minPulse/(maxAngle))*alpha), startSpeed);
}

void loop() {
    static char buffer[128];
    static int buf_idx = 0;

    // Accept commands
    while (Serial.available()) {
        buffer[buf_idx] = Serial.read();

        if (buffer[buf_idx] == '\n') {
            buffer[buf_idx] = 0;
            parseCommand(buffer);
            buf_idx = 0;
        } else {
            buf_idx++;
        }
    }

    // Gradually change prop speed
    if (millis() - last_prop_step > MILLIS_PER_PROP_STEP) {
        adjustProps();
        last_prop_step = millis();
    }
}

// Parse incoming commands
void parseCommand(const char* buf) {
    static char response[128];
    static char command[128];
    static int args[4];

    static int angles[4];
    static int speeds[4];

    sscanf(buf, "%s", command);

    if (strcmp(command, "servos") == 0) {
        sscanf(buf, "servos %d %d %d %d %d %d %d %d", &angles[0], &angles[1], &angles[2], &angles[3], &speeds[0], &speeds[1], &speeds[2], &speeds[3]);
        
        servo1.attach(PIN_SERVO1, 1000, 2000);
        servo2.attach(PIN_SERVO2, 1000, 2000);
        servo3.attach(PIN_SERVO3, 1000, 2000);
        servo4.attach(PIN_SERVO4, 1000, 2000);

        angles[0] = constrain(angles[0], (minAngle-(maxAngle-beta)), (maxAngle-(maxAngle-beta)));
        angles[1] = constrain(angles[1], (minAngle-alpha), (maxAngle-alpha));  
        angles[2] = constrain(angles[2], (minAngle-(maxAngle-beta)), (maxAngle-(maxAngle-beta)));
        angles[3] = constrain(angles[3], (minAngle-alpha),(maxAngle-alpha));

        speeds[0] = constrain(speeds[0], minSpeed, maxSpeed);
        speeds[1] = constrain(speeds[1], minSpeed, maxSpeed);
        speeds[2] = constrain(speeds[2], minSpeed, maxSpeed);
        speeds[3] = constrain(speeds[3], minSpeed, maxSpeed);

        servo1.write((((angles[0]+offset1)*-1+beta)*2), speeds[0]);
        servo2.write((((angles[1]+offset2)+alpha)*2), speeds[1]);
        servo3.write((((angles[2]+offset3)*-1+beta)*2), speeds[2]);
        servo4.write((((angles[3]+offset4)+alpha)*2), speeds[3]);
        
        Serial.println("ack");
    } else if (strcmp(command, "props") == 0) {
        sscanf(buf, "props %d %d %d %d", &args[0], &args[1], &args[2], &args[3]);

        if (args[0] == 0) {
            prop1_target_pwm = 800;
        } else {
            prop1_target_pwm = constrain(thrust2microseconds(args[0] / 1000.0f - 1.0f), 800, 1700);
        }

        if (args[1] == 0) {
            prop2_target_pwm = 800;
        } else {
            prop2_target_pwm = constrain(thrust2microseconds(args[1] / 1000.0f - 1.0f), 800, 1700);
        }

        if (args[2] == 0) {
            prop3_target_pwm = 800;
        } else {
            prop3_target_pwm = constrain(thrust2microseconds(args[2] / 1000.0f - 1.0f), 800, 1700);
        }

        if (args[3] == 0) {
            prop4_target_pwm = 800;
        } else {
            prop4_target_pwm = constrain(thrust2microseconds(args[3] / 1000.0f - 1.0f), 800, 1700);
        }
        
        Serial.println("ack");
    } else if (strcmp(command, "propsraw") == 0) {
        sscanf(buf, "propsraw %d %d %d %d", &args[0], &args[1], &args[2], &args[3]);

        prop1_target_pwm = constrain(args[0], 800, 1700);
        prop2_target_pwm = constrain(args[1], 800, 1700);
        prop3_target_pwm = constrain(args[2], 800, 1700);
        prop4_target_pwm = constrain(args[3], 800, 1700);

        Serial.println("ack");
    } else if (strcmp(command, "retracts") == 0) {
        sscanf(buf, "retracts %d", &args[0]);

        retracts.write(args[0] == 1 ? 135 : 45);
        
        Serial.println("ack");
    } else if (strcmp(command, "pollsonar") == 0) {
        int uS = sonar.ping();
        int cm = uS / US_ROUNDTRIP_CM;

        sprintf(response, "sonar %d", cm);
        Serial.println(response);
    } else if (strcmp(command, "pollbattery") == 0) {
        int v_measure = analogRead(0) * 16.0 * voltage_correct_factor / 1024.0 * 100.0;

        sprintf(response, "battery %d", v_measure);
        Serial.println(response);
    } else if (strcmp(command, "shutdown") == 0) {
        prop1_target_pwm = 800;
        prop2_target_pwm = 800;
        prop3_target_pwm = 800;
        prop4_target_pwm = 800;

        retracts.detach();

        servo1.detach();
        servo2.detach();
        servo3.detach();
        servo4.detach();

        shutdown = true;

        Serial.println("ack");
    } else {
        Serial.println("error");
    }
}

void adjustProps() {
    if (prop1_target_pwm != prop1_pwm) {
        prop1_pwm += constrain(prop1_target_pwm - prop1_pwm, -PWM_PER_PROP_STEP, PWM_PER_PROP_STEP);
        esc1.writeMicroseconds(prop1_pwm);
    } else if (shutdown && !prop1_detached) {
        esc1.detach();
        prop1_detached = true;
    }

    if (prop2_target_pwm != prop2_pwm) {
        prop2_pwm += constrain(prop2_target_pwm - prop2_pwm, -PWM_PER_PROP_STEP, PWM_PER_PROP_STEP);
        esc2.writeMicroseconds(prop2_pwm);
    } else if (shutdown && !prop2_detached) {
        esc2.detach();
        prop2_detached = true;
    }

    if (prop3_target_pwm != prop3_pwm) {
        prop3_pwm += constrain(prop3_target_pwm - prop3_pwm, -PWM_PER_PROP_STEP, PWM_PER_PROP_STEP);
        esc3.writeMicroseconds(prop3_pwm);
    } else if (shutdown && !prop3_detached) {
        esc3.detach();
        prop3_detached = true;
    }

    if (prop4_target_pwm != prop4_pwm) {
        prop4_pwm += constrain(prop4_target_pwm - prop4_pwm, -PWM_PER_PROP_STEP, PWM_PER_PROP_STEP);
        esc4.writeMicroseconds(prop4_pwm);
    } else if (shutdown && !prop4_detached) {
        esc4.detach();
        prop4_detached = true;
    }
}